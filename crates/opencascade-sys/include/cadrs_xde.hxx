// STEP and IGES through OpenCASCADE's XDE (XCAF documents), for cadrs's Import and Export
// (cadrs P3F.2): the assembly structure (parts, their occurrences and placements) and the
// product names, which the plain STEPControl reader and writer don't give.
//
// Reading walks the XCAF document's free shapes: an assembly label's components are followed
// (their locations composed), and every non-assembly shape label is a part, listed once however
// many times it is used. Each use is an occurrence: the part, its placement in the file's
// coordinates, and the component's name. Lengths come back in millimetres.
//
// Writing builds an XCAF document: parts (with names), and, when instances are added, one
// assembly holding them at their placements.
//
// Every function catches OpenCASCADE exceptions (the trycatch in cadrs_safe.hxx).
#pragma once

#include "cadrs_safe.hxx"

#include <IGESCAFControl_Reader.hxx>
#include <IGESCAFControl_Writer.hxx>
#include <IGESControl_Controller.hxx>
#include <Interface_Static.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <ShapeFix_Shape.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Trsf.hxx>

#include <string>
#include <vector>

// A file read through XDE.
class CadrsXde {
public:
  std::string name;
  std::vector<TopoDS_Shape> parts;
  std::vector<std::string> part_names;
  std::vector<int32_t> occ_part;
  std::vector<gp_Trsf> occ_trsf;
  std::vector<std::string> occ_names;
};

// An XCAF document being written.
class CadrsXdeWriter {
public:
  Handle(TDocStd_Document) doc;
  Handle(XCAFDoc_ShapeTool) tool;
  std::string name;
  TDF_Label assembly;
  bool has_assembly = false;
  std::vector<TDF_Label> parts;
};

inline std::string cadrs_label_name(const TDF_Label &label) {
  Handle(TDataStd_Name) n;
  if (label.FindAttribute(TDataStd_Name::GetID(), n)) {
    // UTF-8 (a replacement character of 0 converts rather than replaces).
    TCollection_AsciiString s(n->Get());
    return std::string(s.ToCString());
  }
  return std::string();
}

inline Handle(TDocStd_Document) cadrs_xde_new_document() {
  Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
  Handle(TDocStd_Document) doc;
  app->NewDocument("MDTV-XCAF", doc);
  // Millimetres (the unit is given in metres).
  XCAFDoc_DocumentTool::SetLengthUnit(doc, 0.001);
  return doc;
}

inline void cadrs_xde_walk(const TDF_Label &label, const gp_Trsf &at, const std::string &occ_name,
                           CadrsXde &out, std::vector<TDF_Label> &seen, int depth) {
  if (depth > 64) {
    throw Standard_Failure("the assembly is nested too deeply");
  }
  if (XCAFDoc_ShapeTool::IsAssembly(label)) {
    TDF_LabelSequence comps;
    XCAFDoc_ShapeTool::GetComponents(label, comps, Standard_False);
    for (Standard_Integer i = 1; i <= comps.Length(); i++) {
      const TDF_Label &c = comps.Value(i);
      TDF_Label ref;
      if (!XCAFDoc_ShapeTool::GetReferredShape(c, ref)) {
        continue;
      }
      gp_Trsf t = at * XCAFDoc_ShapeTool::GetLocation(c).Transformation();
      cadrs_xde_walk(ref, t, cadrs_label_name(c), out, seen, depth + 1);
    }
    return;
  }
  int32_t index = -1;
  for (size_t k = 0; k < seen.size(); k++) {
    if (seen[k].IsEqual(label)) {
      index = (int32_t)k;
      break;
    }
  }
  if (index < 0) {
    TopoDS_Shape s = XCAFDoc_ShapeTool::GetShape(label);
    if (s.IsNull()) {
      return;
    }
    index = (int32_t)out.parts.size();
    out.parts.push_back(s);
    out.part_names.push_back(cadrs_label_name(label));
    seen.push_back(label);
  }
  out.occ_part.push_back(index);
  out.occ_trsf.push_back(at);
  out.occ_names.push_back(occ_name);
}

inline std::unique_ptr<CadrsXde> cadrs_read_xde(rust::Str path, bool iges) {
  Handle(TDocStd_Document) doc = cadrs_xde_new_document();
  std::string p(path);
  if (iges) {
    IGESCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    reader.SetColorMode(Standard_False);
    reader.SetLayerMode(Standard_False);
    if (reader.ReadFile(p.c_str()) != IFSelect_RetDone) {
      throw Standard_Failure("the file could not be read as IGES");
    }
    if (!reader.Transfer(doc)) {
      throw Standard_Failure("the IGES file holds no shapes cadrs can use");
    }
  } else {
    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    reader.SetColorMode(Standard_False);
    reader.SetLayerMode(Standard_False);
    reader.SetPropsMode(Standard_False);
    reader.SetGDTMode(Standard_False);
    reader.SetMatMode(Standard_False);
    reader.SetViewMode(Standard_False);
    if (reader.ReadFile(p.c_str()) != IFSelect_RetDone) {
      throw Standard_Failure("the file could not be read as STEP");
    }
    if (!reader.Transfer(doc)) {
      throw Standard_Failure("the STEP file holds no shapes cadrs can use");
    }
  }
  Handle(XCAFDoc_ShapeTool) tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
  TDF_LabelSequence free;
  tool->GetFreeShapes(free);
  std::unique_ptr<CadrsXde> out(new CadrsXde());
  std::vector<TDF_Label> seen;
  for (Standard_Integer i = 1; i <= free.Length(); i++) {
    const TDF_Label &l = free.Value(i);
    std::string n = cadrs_label_name(l);
    if (out->name.empty()) {
      out->name = n;
    }
    cadrs_xde_walk(l, gp_Trsf(), n, *out, seen, 0);
  }
  XCAFApp_Application::GetApplication()->Close(doc);
  return out;
}

inline rust::String cadrs_xde_name(const CadrsXde &x) { return rust::String::lossy(x.name); }
inline int32_t cadrs_xde_part_count(const CadrsXde &x) { return (int32_t)x.parts.size(); }
inline std::unique_ptr<TopoDS_Shape> cadrs_xde_part_shape(const CadrsXde &x, int32_t i) {
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(x.parts.at(i)));
}
inline rust::String cadrs_xde_part_name(const CadrsXde &x, int32_t i) {
  return rust::String::lossy(x.part_names.at(i));
}
inline int32_t cadrs_xde_occurrence_count(const CadrsXde &x) { return (int32_t)x.occ_part.size(); }
inline int32_t cadrs_xde_occurrence_part(const CadrsXde &x, int32_t i) { return x.occ_part.at(i); }
inline rust::String cadrs_xde_occurrence_name(const CadrsXde &x, int32_t i) {
  return rust::String::lossy(x.occ_names.at(i));
}
// The placement as a 3×4 matrix, rows first: [R | t].
inline void cadrs_xde_occurrence_matrix(const CadrsXde &x, int32_t i, rust::Slice<double> out) {
  const gp_Trsf &t = x.occ_trsf.at(i);
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 4; c++) {
      if ((size_t)(r * 4 + c) < out.size()) {
        out[r * 4 + c] = t.Value(r + 1, c + 1);
      }
    }
  }
}

inline std::unique_ptr<CadrsXdeWriter> cadrs_xde_writer_new(rust::Str name) {
  std::unique_ptr<CadrsXdeWriter> w(new CadrsXdeWriter());
  w->doc = cadrs_xde_new_document();
  w->tool = XCAFDoc_DocumentTool::ShapeTool(w->doc->Main());
  w->name = std::string(name);
  return w;
}

inline int32_t cadrs_xde_writer_add_part(CadrsXdeWriter &w, const TopoDS_Shape &shape, rust::Str name) {
  TDF_Label l = w.tool->AddShape(shape, Standard_False);
  std::string n(name);
  TDataStd_Name::Set(l, TCollection_ExtendedString(n.c_str(), Standard_True));
  w.parts.push_back(l);
  return (int32_t)w.parts.size() - 1;
}

inline void cadrs_xde_writer_add_instance(CadrsXdeWriter &w, int32_t part, rust::Slice<const double> m,
                                          rust::Str name) {
  if (m.size() < 12) {
    throw Standard_Failure("a placement needs 12 numbers");
  }
  if (!w.has_assembly) {
    w.assembly = w.tool->NewShape();
    TDataStd_Name::Set(w.assembly, TCollection_ExtendedString(w.name.c_str(), Standard_True));
    w.has_assembly = true;
  }
  gp_Trsf t;
  t.SetValues(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
  TDF_Label c = w.tool->AddComponent(w.assembly, w.parts.at(part), TopLoc_Location(t));
  std::string n(name);
  if (!c.IsNull() && !n.empty()) {
    TDataStd_Name::Set(c, TCollection_ExtendedString(n.c_str(), Standard_True));
  }
}

inline void cadrs_xde_writer_write(CadrsXdeWriter &w, rust::Str path, bool iges) {
  w.tool->UpdateAssemblies();
  std::string p(path);
  if (iges) {
    // Solids as solids (MSBO), so they read back as solids. The writer reads the mode when it
    // is made, and the setting exists once the IGES controller is initialised.
    IGESControl_Controller::Init();
    Interface_Static::SetIVal("write.iges.brep.mode", 1);
    IGESCAFControl_Writer writer;
    writer.SetNameMode(Standard_True);
    writer.SetColorMode(Standard_False);
    if (!writer.Transfer(w.doc)) {
      throw Standard_Failure("the shapes could not be written as IGES");
    }
    if (!writer.Write(p.c_str())) {
      throw Standard_Failure("the IGES file could not be written");
    }
  } else {
    STEPCAFControl_Writer writer;
    writer.SetNameMode(Standard_True);
    writer.SetColorMode(Standard_False);
    writer.SetLayerMode(Standard_False);
    if (!writer.Transfer(w.doc, STEPControl_AsIs)) {
      throw Standard_Failure("the shapes could not be written as STEP");
    }
    if (writer.Write(p.c_str()) != IFSelect_RetDone) {
      throw Standard_Failure("the STEP file could not be written");
    }
  }
  XCAFApp_Application::GetApplication()->Close(w.doc);
}

// OpenCASCADE's shape healing (ShapeFix_Shape) of an imported shape: small gaps, wrong
// orientations, missing pcurves and the like.
inline std::unique_ptr<TopoDS_Shape> cadrs_shape_fix(const TopoDS_Shape &shape) {
  Handle(ShapeFix_Shape) fixer = new ShapeFix_Shape(shape);
  fixer->Perform();
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(fixer->Shape()));
}
