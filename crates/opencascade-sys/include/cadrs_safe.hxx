// Exception-safe wrappers for the operations cadrs uses (cadrs branch).
//
// Every function here is declared with a `Result` return type in `src/cadrs_safe.rs`, so cxx
// wraps the call in `rust::behavior::trycatch`. The default trycatch only catches
// `std::exception`, but OpenCASCADE throws `Standard_Failure` (which is not one), so an OCCT
// error would call std::terminate and abort the process. The trycatch below catches OCCT
// failures as well and turns them into a Rust `Err` carrying the exception's type and message.
//
// Each function does the whole operation, including reading the builder's result, inside the
// call: most OCCT builders throw `StdFail_NotDone` when the result is read after a failure.
#pragma once

#include <Standard_Failure.hxx>
#include <exception>
#include <string>

namespace rust {
namespace behavior {
template <typename Try, typename Fail> static void trycatch(Try &&func, Fail &&fail) noexcept try {
  func();
} catch (const Standard_Failure &e) {
  std::string msg = e.DynamicType()->Name();
  const char *what = e.GetMessageString();
  if (what != nullptr && *what != '\0') {
    msg += ": ";
    msg += what;
  }
  fail(msg.c_str());
} catch (const std::exception &e) {
  fail(e.what());
} catch (...) {
  fail("unknown OpenCASCADE exception");
}
} // namespace behavior
} // namespace rust

#include "rust/cxx.h"
#include <memory>
#include <vector>

#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <Poly_Connect.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepTools_History.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <BRepBuilderAPI_MakeShape.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Vertex.hxx>
#include <algorithm>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepLProp_SLProps.hxx>
#include <Geom2d_Curve.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepBndLib.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Lin.hxx>

// ---------------------------------------------------------------------------------------------
// Edges

inline std::unique_ptr<TopoDS_Edge> cadrs_edge_from(BRepBuilderAPI_MakeEdge &make) {
  if (!make.IsDone()) {
    throw std::runtime_error("edge construction failed (error " + std::to_string((int)make.Error()) + ")");
  }
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(make.Edge()));
}

inline std::unique_ptr<TopoDS_Edge> cadrs_edge_segment(double ax, double ay, double az, double bx, double by,
                                                      double bz) {
  BRepBuilderAPI_MakeEdge make(gp_Pnt(ax, ay, az), gp_Pnt(bx, by, bz));
  return cadrs_edge_from(make);
}

inline std::unique_ptr<TopoDS_Edge> cadrs_edge_arc3(double ax, double ay, double az, double bx, double by,
                                                   double bz, double cx, double cy, double cz) {
  GC_MakeArcOfCircle arc(gp_Pnt(ax, ay, az), gp_Pnt(bx, by, bz), gp_Pnt(cx, cy, cz));
  if (!arc.IsDone()) {
    throw std::runtime_error("arc through three points failed (collinear points?)");
  }
  BRepBuilderAPI_MakeEdge make(arc.Value());
  return cadrs_edge_from(make);
}

/// A (rational) Bezier curve through `poles` (x, y, z triples) with a weight per pole: with three
/// poles, a conic (cadrs P3.6: conic and curvature fillet sections).
inline std::unique_ptr<TopoDS_Edge> cadrs_edge_bezier(rust::Slice<const double> poles,
                                                      rust::Slice<const double> weights) {
  const int n = (int)weights.size();
  if (n < 2 || (int)poles.size() != 3 * n) {
    throw std::runtime_error("a Bezier curve needs at least two poles and a weight per pole");
  }
  TColgp_Array1OfPnt p(1, n);
  TColStd_Array1OfReal w(1, n);
  for (int i = 0; i < n; ++i) {
    p.SetValue(i + 1, gp_Pnt(poles[3 * i], poles[3 * i + 1], poles[3 * i + 2]));
    w.SetValue(i + 1, weights[i]);
  }
  Handle(Geom_BezierCurve) curve = new Geom_BezierCurve(p, w);
  BRepBuilderAPI_MakeEdge make(curve);
  return cadrs_edge_from(make);
}

/// A full circle about `normal` (counter-clockwise seen from it), starting on `x_dir`.
inline std::unique_ptr<TopoDS_Edge> cadrs_edge_circle(double cx, double cy, double cz, double nx, double ny,
                                                     double nz, double xx, double xy, double xz, double radius) {
  gp_Circ circ(gp_Ax2(gp_Pnt(cx, cy, cz), gp_Dir(nx, ny, nz), gp_Dir(xx, xy, xz)), radius);
  BRepBuilderAPI_MakeEdge make(circ);
  return cadrs_edge_from(make);
}

/// An ellipse about `normal` with its major axis along `x_dir` (`major` >= `minor`): the whole
/// ellipse if `full`, else the arc from the point `p1` to `p2`, counter-clockwise seen from
/// `normal` (the points are projected onto the ellipse and become the edge's vertices).
inline std::unique_ptr<TopoDS_Edge> cadrs_edge_ellipse(double cx, double cy, double cz, double nx, double ny,
                                                      double nz, double xx, double xy, double xz, double major,
                                                      double minor, bool full, double ax, double ay, double az,
                                                      double bx, double by, double bz) {
  gp_Elips elips(gp_Ax2(gp_Pnt(cx, cy, cz), gp_Dir(nx, ny, nz), gp_Dir(xx, xy, xz)), major, minor);
  if (full) {
    BRepBuilderAPI_MakeEdge make(elips);
    return cadrs_edge_from(make);
  }
  BRepBuilderAPI_MakeEdge make(elips, gp_Pnt(ax, ay, az), gp_Pnt(bx, by, bz));
  return cadrs_edge_from(make);
}

/// Points along the edge: the polygon the mesher stored on a face of `shape` that contains the
/// edge (so edges line up with the shaded triangles), else a tangential-deflection sampling.
inline std::unique_ptr<std::vector<double>> cadrs_edge_polyline(const TopoDS_Edge &edge, const TopoDS_Shape &shape,
                                                               double angular, double deflection) {
  std::unique_ptr<std::vector<double>> out(new std::vector<double>());
  for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
    const TopoDS_Face &face = TopoDS::Face(faces.Current());
    for (TopExp_Explorer edges(face, TopAbs_EDGE); edges.More(); edges.Next()) {
      if (!edges.Current().IsSame(edge)) {
        continue;
      }
      TopLoc_Location loc;
      Handle(Poly_Triangulation) tris = BRep_Tool::Triangulation(face, loc);
      if (tris.IsNull()) {
        break;
      }
      Handle(Poly_PolygonOnTriangulation) poly =
          BRep_Tool::PolygonOnTriangulation(TopoDS::Edge(edges.Current()), tris, loc);
      if (poly.IsNull()) {
        break;
      }
      const gp_Trsf trsf = loc.Transformation();
      const TColStd_Array1OfInteger &nodes = poly->Nodes();
      for (Standard_Integer i = nodes.Lower(); i <= nodes.Upper(); ++i) {
        gp_Pnt p = tris->Node(nodes(i)).Transformed(trsf);
        out->push_back(p.X());
        out->push_back(p.Y());
        out->push_back(p.Z());
      }
      return out;
    }
  }
  BRepAdaptor_Curve curve(edge);
  GCPnts_TangentialDeflection sampler(curve, angular, deflection);
  for (Standard_Integer i = 1; i <= sampler.NbPoints(); ++i) {
    gp_Pnt p = sampler.Value(i);
    out->push_back(p.X());
    out->push_back(p.Y());
    out->push_back(p.Z());
  }
  return out;
}

// ---------------------------------------------------------------------------------------------
// Wires and faces

inline std::unique_ptr<BRepBuilderAPI_MakeWire> cadrs_wire_builder() {
  return std::unique_ptr<BRepBuilderAPI_MakeWire>(new BRepBuilderAPI_MakeWire());
}

inline void cadrs_wire_add(BRepBuilderAPI_MakeWire &builder, const TopoDS_Edge &edge) {
  builder.Add(edge);
  if (!builder.IsDone()) {
    throw std::runtime_error("the edge does not connect to the wire (error " + std::to_string((int)builder.Error()) +
                             ")");
  }
}

inline std::unique_ptr<TopoDS_Wire> cadrs_wire_build(BRepBuilderAPI_MakeWire &builder) {
  if (!builder.IsDone()) {
    throw std::runtime_error("wire construction failed");
  }
  return std::unique_ptr<TopoDS_Wire>(new TopoDS_Wire(builder.Wire()));
}

/// A planar face bounded by `outer`.
inline std::unique_ptr<TopoDS_Face> cadrs_face_from_wire(const TopoDS_Wire &outer) {
  BRepBuilderAPI_MakeFace make(outer, Standard_True);
  if (!make.IsDone()) {
    throw std::runtime_error("planar face construction failed (error " + std::to_string((int)make.Error()) + ")");
  }
  return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(make.Face()));
}

/// `face` with another hole.
inline std::unique_ptr<TopoDS_Face> cadrs_face_add_hole(const TopoDS_Face &face, const TopoDS_Wire &hole) {
  BRepBuilderAPI_MakeFace make(face);
  make.Add(hole);
  if (!make.IsDone()) {
    throw std::runtime_error("adding a hole to the face failed (error " + std::to_string((int)make.Error()) + ")");
  }
  return std::unique_ptr<TopoDS_Face>(new TopoDS_Face(make.Face()));
}

/// The surface type of a face (`GeomAbs_SurfaceType` as an integer: 0 plane, 1 cylinder,
/// 2 cone, 3 sphere, 4 torus, others above).
inline int32_t cadrs_face_surface_type(const TopoDS_Face &face) {
  BRepAdaptor_Surface surface(face);
  return (int32_t)surface.GetType();
}

// ---------------------------------------------------------------------------------------------
// Solids

inline std::unique_ptr<TopoDS_Shape> cadrs_prism(const TopoDS_Shape &shape, double dx, double dy, double dz) {
  BRepPrimAPI_MakePrism make(shape, gp_Vec(dx, dy, dz));
  if (!make.IsDone()) {
    throw std::runtime_error("extrusion failed");
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(make.Shape()));
}

inline std::unique_ptr<TopoDS_Shape> cadrs_revol(const TopoDS_Shape &shape, double ox, double oy, double oz,
                                                double dx, double dy, double dz, double angle) {
  BRepPrimAPI_MakeRevol make(shape, gp_Ax1(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz)), angle);
  if (!make.IsDone()) {
    throw std::runtime_error("revolve failed");
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(make.Shape()));
}

/// A boolean: `op` 0 = fuse (union), 1 = cut (subtract), 2 = common (intersect).
inline std::unique_ptr<TopoDS_Shape> cadrs_boolean(const TopoDS_Shape &a, const TopoDS_Shape &b, int32_t op) {
  std::unique_ptr<BRepAlgoAPI_BooleanOperation> algo;
  switch (op) {
  case 0:
    algo.reset(new BRepAlgoAPI_Fuse(a, b));
    break;
  case 1:
    algo.reset(new BRepAlgoAPI_Cut(a, b));
    break;
  case 2:
    algo.reset(new BRepAlgoAPI_Common(a, b));
    break;
  default:
    throw std::runtime_error("unknown boolean operation");
  }
  if (!algo->IsDone() || algo->HasErrors()) {
    throw std::runtime_error("boolean operation failed");
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(algo->Shape()));
}

/// Merges coplanar and cocylindrical neighbours into single faces (and collinear edges).
inline std::unique_ptr<TopoDS_Shape> cadrs_unify(const TopoDS_Shape &shape) {
  ShapeUpgrade_UnifySameDomain unify(shape, Standard_True, Standard_True, Standard_True);
  unify.AllowInternalEdges(Standard_False);
  unify.Build();
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(unify.Shape()));
}

/// Constant-radius fillet of `edges` (a list of edges of `shape`).
inline std::unique_ptr<TopoDS_Shape> cadrs_fillet(const TopoDS_Shape &shape, const TopTools_ListOfShape &edges,
                                                 double radius) {
  BRepFilletAPI_MakeFillet make(shape);
  for (TopTools_ListOfShape::Iterator it(edges); it.More(); it.Next()) {
    make.Add(radius, TopoDS::Edge(it.Value()));
  }
  make.Build();
  if (!make.IsDone()) {
    throw std::runtime_error("fillet failed");
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(make.Shape()));
}

/// Chamfer of `edges`. `mode` 0: equal distance `d1`; 1: distances `d1` (on the matching face in
/// `faces`) and `d2`; 2: distance `d1` (on the face) and angle `d2` (radians). `faces` is parallel
/// to `edges` for modes 1 and 2.
inline std::unique_ptr<TopoDS_Shape> cadrs_chamfer(const TopoDS_Shape &shape, const TopTools_ListOfShape &edges,
                                                  const TopTools_ListOfShape &faces, int32_t mode, double d1,
                                                  double d2) {
  BRepFilletAPI_MakeChamfer make(shape);
  TopTools_ListOfShape::Iterator f(faces);
  for (TopTools_ListOfShape::Iterator it(edges); it.More(); it.Next()) {
    const TopoDS_Edge &edge = TopoDS::Edge(it.Value());
    if (mode == 0) {
      make.Add(d1, edge);
      continue;
    }
    if (!f.More()) {
      throw std::runtime_error("chamfer: a face is needed for each edge");
    }
    const TopoDS_Face &face = TopoDS::Face(f.Value());
    f.Next();
    if (mode == 1) {
      make.Add(d1, d2, edge, face);
    } else {
      make.AddDA(d1, d2, edge, face);
    }
  }
  make.Build();
  if (!make.IsDone()) {
    throw std::runtime_error("chamfer failed");
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(make.Shape()));
}

/// Hollows `shape`, removing `faces` (negative `offset` = inward).
inline std::unique_ptr<TopoDS_Shape> cadrs_thick_solid(const TopoDS_Shape &shape, const TopTools_ListOfShape &faces,
                                                      double offset, double tolerance) {
  BRepOffsetAPI_MakeThickSolid make;
  make.MakeThickSolidByJoin(shape, faces, offset, tolerance);
  if (!make.IsDone()) {
    throw std::runtime_error("shell failed");
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(make.Shape()));
}

// ---------------------------------------------------------------------------------------------
// Modeling history (persistent naming, cadrs P3.2)
//
// The `*_h` operations also write what the operation did to its inputs into `hist`, as indices
// into the result's faces (`TopExp::MapShapes(result, TopAbs_FACE)` order, 0-based). `hist` has
// five sections, each a count `n` followed by `n` entries; each entry is a count `k` followed by
// `k` face indices:
//   1. faces:    every face of the inputs (in input order, MapShapes order within an input):
//                the result faces it became (`Modified`, or itself if kept; k = 0: deleted);
//   2. edges:    every edge of the inputs: the result faces generated from it (`Generated`);
//   3. vertices: every vertex of the inputs: the result faces generated from it;
//   4. first:    one entry, the faces of `FirstShape()` (a sweep's start cap; k = 0 otherwise);
//   5. last:     one entry, the faces of `LastShape()`.

/// One entry: the distinct result faces of `list` that are faces.
inline void cadrs_hist_entry(const TopTools_ListOfShape &list, const TopTools_IndexedMapOfShape &result,
                             std::vector<int32_t> &out) {
  std::vector<int32_t> idx;
  for (TopTools_ListOfShape::Iterator it(list); it.More(); it.Next()) {
    if (it.Value().ShapeType() != TopAbs_FACE) {
      continue;
    }
    const int32_t k = (int32_t)result.FindIndex(it.Value()) - 1;
    if (k >= 0 && std::find(idx.begin(), idx.end(), k) == idx.end()) {
      idx.push_back(k);
    }
  }
  out.push_back((int32_t)idx.size());
  out.insert(out.end(), idx.begin(), idx.end());
}

/// One entry: the result faces among the faces of `shape`.
inline void cadrs_hist_faces_of(const TopoDS_Shape *shape, const TopTools_IndexedMapOfShape &result,
                                std::vector<int32_t> &out) {
  TopTools_ListOfShape list;
  if (shape != nullptr && !shape->IsNull()) {
    for (TopExp_Explorer ex(*shape, TopAbs_FACE); ex.More(); ex.Next()) {
      list.Append(ex.Current());
    }
  }
  cadrs_hist_entry(list, result, out);
}

/// Writes the history of `make` for `inputs` into `out` (see above).
template <typename Make>
inline void cadrs_history(Make &make, const std::vector<const TopoDS_Shape *> &inputs, const TopoDS_Shape &result,
                          const TopoDS_Shape *first, const TopoDS_Shape *last, std::vector<int32_t> &out) {
  TopTools_IndexedMapOfShape res;
  TopExp::MapShapes(result, TopAbs_FACE, res);
  const TopAbs_ShapeEnum kinds[3] = {TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX};
  for (int kind = 0; kind < 3; ++kind) {
    std::vector<TopoDS_Shape> subs;
    for (const TopoDS_Shape *input : inputs) {
      TopTools_IndexedMapOfShape map;
      TopExp::MapShapes(*input, kinds[kind], map);
      for (Standard_Integer i = 1; i <= map.Extent(); ++i) {
        subs.push_back(map(i));
      }
    }
    out.push_back((int32_t)subs.size());
    for (const TopoDS_Shape &sub : subs) {
      if (kind != 0) {
        cadrs_hist_entry(make.Generated(sub), res, out);
      } else if (make.IsDeleted(sub)) {
        out.push_back(0);
      } else if (make.Modified(sub).IsEmpty()) {
        // Kept as it is (or gone without being reported deleted).
        TopTools_ListOfShape self;
        self.Append(sub);
        cadrs_hist_entry(self, res, out);
      } else {
        cadrs_hist_entry(make.Modified(sub), res, out);
      }
    }
  }
  out.push_back(1);
  cadrs_hist_faces_of(first, res, out);
  out.push_back(1);
  cadrs_hist_faces_of(last, res, out);
}

inline std::unique_ptr<TopoDS_Shape> cadrs_prism_h(const TopoDS_Shape &shape, double dx, double dy, double dz,
                                                  std::vector<int32_t> &hist) {
  BRepPrimAPI_MakePrism make(shape, gp_Vec(dx, dy, dz));
  if (!make.IsDone()) {
    throw std::runtime_error("extrusion failed");
  }
  const TopoDS_Shape result = make.Shape();
  const TopoDS_Shape first = make.FirstShape();
  const TopoDS_Shape last = make.LastShape();
  cadrs_history(make, {&shape}, result, &first, &last, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

inline std::unique_ptr<TopoDS_Shape> cadrs_revol_h(const TopoDS_Shape &shape, double ox, double oy, double oz,
                                                  double dx, double dy, double dz, double angle,
                                                  std::vector<int32_t> &hist) {
  BRepPrimAPI_MakeRevol make(shape, gp_Ax1(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz)), angle);
  if (!make.IsDone()) {
    throw std::runtime_error("revolve failed");
  }
  const TopoDS_Shape result = make.Shape();
  const TopoDS_Shape first = make.FirstShape();
  const TopoDS_Shape last = make.LastShape();
  cadrs_history(make, {&shape}, result, &first, &last, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// [`cadrs_boolean`] with history (inputs: `a`, then `b`).
inline std::unique_ptr<TopoDS_Shape> cadrs_boolean_h(const TopoDS_Shape &a, const TopoDS_Shape &b, int32_t op,
                                                    std::vector<int32_t> &hist) {
  std::unique_ptr<BRepAlgoAPI_BooleanOperation> algo;
  switch (op) {
  case 0:
    algo.reset(new BRepAlgoAPI_Fuse(a, b));
    break;
  case 1:
    algo.reset(new BRepAlgoAPI_Cut(a, b));
    break;
  case 2:
    algo.reset(new BRepAlgoAPI_Common(a, b));
    break;
  default:
    throw std::runtime_error("unknown boolean operation");
  }
  if (!algo->IsDone() || algo->HasErrors()) {
    throw std::runtime_error("boolean operation failed");
  }
  const TopoDS_Shape result = algo->Shape();
  cadrs_history(*algo, {&a, &b}, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// [`cadrs_fillet`] with history.
inline std::unique_ptr<TopoDS_Shape> cadrs_fillet_h(const TopoDS_Shape &shape, const TopTools_ListOfShape &edges,
                                                   double radius, std::vector<int32_t> &hist) {
  BRepFilletAPI_MakeFillet make(shape);
  for (TopTools_ListOfShape::Iterator it(edges); it.More(); it.Next()) {
    make.Add(radius, TopoDS::Edge(it.Value()));
  }
  make.Build();
  if (!make.IsDone()) {
    throw std::runtime_error("fillet failed");
  }
  const TopoDS_Shape result = make.Shape();
  cadrs_history(make, {&shape}, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// [`cadrs_chamfer`] with history.
inline std::unique_ptr<TopoDS_Shape> cadrs_chamfer_h(const TopoDS_Shape &shape, const TopTools_ListOfShape &edges,
                                                    const TopTools_ListOfShape &faces, int32_t mode, double d1,
                                                    double d2, std::vector<int32_t> &hist) {
  BRepFilletAPI_MakeChamfer make(shape);
  TopTools_ListOfShape::Iterator f(faces);
  for (TopTools_ListOfShape::Iterator it(edges); it.More(); it.Next()) {
    const TopoDS_Edge &edge = TopoDS::Edge(it.Value());
    if (mode == 0) {
      make.Add(d1, edge);
      continue;
    }
    if (!f.More()) {
      throw std::runtime_error("chamfer: a face is needed for each edge");
    }
    const TopoDS_Face &face = TopoDS::Face(f.Value());
    f.Next();
    if (mode == 1) {
      make.Add(d1, d2, edge, face);
    } else {
      make.AddDA(d1, d2, edge, face);
    }
  }
  make.Build();
  if (!make.IsDone()) {
    throw std::runtime_error("chamfer failed");
  }
  const TopoDS_Shape result = make.Shape();
  cadrs_history(make, {&shape}, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// [`cadrs_thick_solid`] with history.
inline std::unique_ptr<TopoDS_Shape> cadrs_thick_solid_h(const TopoDS_Shape &shape, const TopTools_ListOfShape &faces,
                                                        double offset, double tolerance, std::vector<int32_t> &hist) {
  BRepOffsetAPI_MakeThickSolid make;
  make.MakeThickSolidByJoin(shape, faces, offset, tolerance);
  if (!make.IsDone()) {
    throw std::runtime_error("shell failed");
  }
  const TopoDS_Shape result = make.Shape();
  cadrs_history(make, {&shape}, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

// ---------------------------------------------------------------------------------------------
// Topology queries (indices in `TopExp::MapShapes` order, 0-based)

/// The number of distinct faces, edges and vertices of `shape` (the index ranges used above).
inline void cadrs_counts(const TopoDS_Shape &shape, std::vector<int32_t> &out) {
  const TopAbs_ShapeEnum kinds[3] = {TopAbs_FACE, TopAbs_EDGE, TopAbs_VERTEX};
  for (TopAbs_ShapeEnum kind : kinds) {
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(shape, kind, map);
    out.push_back((int32_t)map.Extent());
  }
}

/// Per edge of `shape`, 17 numbers: its start, end and middle points (xyz each), the unit
/// tangents at its start and end (in the direction the edge runs), its exact length, and its curve
/// type (`GeomAbs_CurveType`: 0 line, 1 circle, 2 ellipse, ...; -1 for a degenerate edge, such
/// as a cone's apex, which has zero length and zero tangents).
inline void cadrs_edges_info(const TopoDS_Shape &shape, std::vector<double> &out) {
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(shape, TopAbs_EDGE, edges);
  for (Standard_Integer i = 1; i <= edges.Extent(); ++i) {
    const TopoDS_Edge &edge = TopoDS::Edge(edges(i));
    double v[17] = {0};
    if (BRep_Tool::Degenerated(edge)) {
      const gp_Pnt p = BRep_Tool::Pnt(TopExp::FirstVertex(edge));
      for (int k = 0; k < 3; ++k) {
        v[3 * k] = p.X();
        v[3 * k + 1] = p.Y();
        v[3 * k + 2] = p.Z();
      }
      v[16] = -1.0;
      out.insert(out.end(), v, v + 17);
      continue;
    }
    BRepAdaptor_Curve c(edge);
    const double t0 = c.FirstParameter(), t1 = c.LastParameter();
    gp_Pnt p0, p1, pm;
    gp_Vec d0, d1, dm;
    c.D1(t0, p0, d0);
    c.D1(t1, p1, d1);
    c.D1((t0 + t1) / 2.0, pm, dm);
    if (edge.Orientation() == TopAbs_REVERSED) {
      std::swap(p0, p1);
      std::swap(d0, d1);
      d0.Reverse();
      d1.Reverse();
    }
    if (d0.Magnitude() > 1e-300) {
      d0.Normalize();
    }
    if (d1.Magnitude() > 1e-300) {
      d1.Normalize();
    }
    const gp_Pnt ps[3] = {p0, p1, pm};
    for (int k = 0; k < 3; ++k) {
      v[3 * k] = ps[k].X();
      v[3 * k + 1] = ps[k].Y();
      v[3 * k + 2] = ps[k].Z();
    }
    v[9] = d0.X();
    v[10] = d0.Y();
    v[11] = d0.Z();
    v[12] = d1.X();
    v[13] = d1.Y();
    v[14] = d1.Z();
    v[15] = GCPnts_AbscissaPoint::Length(c);
    v[16] = (double)c.GetType();
    out.insert(out.end(), v, v + 17);
  }
}

/// Per edge of `shape`: a count and the indices of the faces around it (a seam edge lists its
/// face once).
inline void cadrs_edge_faces(const TopoDS_Shape &shape, std::vector<int32_t> &out) {
  TopTools_IndexedMapOfShape faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(shape, TopAbs_EDGE, edges);
  TopTools_IndexedDataMapOfShapeListOfShape around;
  TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_EDGE, TopAbs_FACE, around);
  for (Standard_Integer i = 1; i <= edges.Extent(); ++i) {
    TopTools_ListOfShape list;
    const Standard_Integer j = around.FindIndex(edges(i));
    if (j > 0) {
      list = around(j);
    }
    cadrs_hist_entry(list, faces, out);
  }
}

/// Per vertex of `shape`: its point (xyz, into `points`), and a count and the indices of the
/// (non-degenerate) edges that end at it (into `edges_out`).
inline void cadrs_vertices(const TopoDS_Shape &shape, std::vector<double> &points, std::vector<int32_t> &edges_out) {
  TopTools_IndexedMapOfShape vertices;
  TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(shape, TopAbs_EDGE, edges);
  TopTools_IndexedDataMapOfShapeListOfShape around;
  TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, around);
  for (Standard_Integer i = 1; i <= vertices.Extent(); ++i) {
    const gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vertices(i)));
    points.push_back(p.X());
    points.push_back(p.Y());
    points.push_back(p.Z());
    std::vector<int32_t> idx;
    const Standard_Integer j = around.FindIndex(vertices(i));
    if (j > 0) {
      for (TopTools_ListOfShape::Iterator it(around(j)); it.More(); it.Next()) {
        const int32_t k = (int32_t)edges.FindIndex(it.Value()) - 1;
        if (k >= 0 && !BRep_Tool::Degenerated(TopoDS::Edge(it.Value())) &&
            std::find(idx.begin(), idx.end(), k) == idx.end()) {
          idx.push_back(k);
        }
      }
    }
    edges_out.push_back((int32_t)idx.size());
    edges_out.insert(edges_out.end(), idx.begin(), idx.end());
  }
}

// ---------------------------------------------------------------------------------------------
// Meshing

/// Triangulates every face of `shape` (linear deflection in model units, angular in radians).
inline void cadrs_mesh(const TopoDS_Shape &shape, double deflection, double angular) {
  BRepMesh_IncrementalMesh mesh(shape, deflection, Standard_False, angular, Standard_True);
  if (!mesh.IsDone()) {
    throw std::runtime_error("meshing failed");
  }
}

/// The stored triangulation of a meshed face: positions and normals (xyz triples, normals
/// pointing out of the material) and triangle indices (0-based, counter-clockwise seen from
/// outside).
inline void cadrs_face_triangulation(const TopoDS_Face &face, std::vector<double> &positions,
                                     std::vector<double> &normals, std::vector<int32_t> &indices) {
  TopLoc_Location loc;
  Handle(Poly_Triangulation) tris = BRep_Tool::Triangulation(face, loc);
  if (tris.IsNull()) {
    throw std::runtime_error("the face has no triangulation");
  }
  Poly_Connect connect;
  BRepLib_ToolTriangulatedShape::ComputeNormals(face, tris, connect);
  const gp_Trsf trsf = loc.Transformation();
  const bool reversed = face.Orientation() == TopAbs_REVERSED;
  for (Standard_Integer i = 1; i <= tris->NbNodes(); ++i) {
    gp_Pnt p = tris->Node(i).Transformed(trsf);
    positions.push_back(p.X());
    positions.push_back(p.Y());
    positions.push_back(p.Z());
    gp_Dir n = tris->Normal(i).Transformed(trsf);
    if (reversed) {
      n.Reverse();
    }
    normals.push_back(n.X());
    normals.push_back(n.Y());
    normals.push_back(n.Z());
  }
  for (Standard_Integer i = 1; i <= tris->NbTriangles(); ++i) {
    Standard_Integer a, b, c;
    tris->Triangle(i).Get(a, b, c);
    if (reversed) {
      std::swap(b, c);
    }
    indices.push_back(a - 1);
    indices.push_back(b - 1);
    indices.push_back(c - 1);
  }
}

// ---------------------------------------------------------------------------------------------
// Identity

/// A hash that is equal for shapes that are `IsSame` (same TShape and location, any orientation).
inline uint64_t cadrs_shape_hash(const TopoDS_Shape &shape) { return (uint64_t)std::hash<TopoDS_Shape>{}(shape); }

inline bool cadrs_shape_is_same(const TopoDS_Shape &a, const TopoDS_Shape &b) { return a.IsSame(b); }

inline std::unique_ptr<std::vector<double>> cadrs_new_f64_vec() {
  return std::unique_ptr<std::vector<double>>(new std::vector<double>());
}

inline std::unique_ptr<std::vector<int32_t>> cadrs_new_i32_vec() {
  return std::unique_ptr<std::vector<int32_t>>(new std::vector<int32_t>());
}

// ---------------------------------------------------------------------------------------------
// cadrs P3.3: sub-shapes, compounds, thickening, ray and bounding-box queries

/// The `TopAbs_ShapeEnum` for cadrs's kind numbers: 0 solid, 1 shell, 2 face, 3 wire, 4 edge.
inline TopAbs_ShapeEnum cadrs_kind(int32_t kind) {
  switch (kind) {
  case 0:
    return TopAbs_SOLID;
  case 1:
    return TopAbs_SHELL;
  case 2:
    return TopAbs_FACE;
  case 3:
    return TopAbs_WIRE;
  case 4:
    return TopAbs_EDGE;
  default:
    throw std::runtime_error("unknown sub-shape kind");
  }
}

/// The number of distinct sub-shapes of a kind (see `cadrs_kind`), in `TopExp::MapShapes` order.
inline int32_t cadrs_sub_count(const TopoDS_Shape &shape, int32_t kind) {
  TopTools_IndexedMapOfShape map;
  TopExp::MapShapes(shape, cadrs_kind(kind), map);
  return (int32_t)map.Extent();
}

/// The `index`-th (0-based) distinct sub-shape of a kind. It shares its faces, edges and vertices
/// with `shape` (they are `IsSame`).
inline std::unique_ptr<TopoDS_Shape> cadrs_sub_shape(const TopoDS_Shape &shape, int32_t kind, int32_t index) {
  TopTools_IndexedMapOfShape map;
  TopExp::MapShapes(shape, cadrs_kind(kind), map);
  if (index < 0 || index >= map.Extent()) {
    throw std::runtime_error("sub-shape index out of range");
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(map(index + 1)));
}

/// A compound of the shapes (they keep their sub-shapes).
inline std::unique_ptr<TopoDS_Shape> cadrs_compound(const TopTools_ListOfShape &shapes) {
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (TopTools_ListOfShape::Iterator it(shapes); it.More(); it.Next()) {
    builder.Add(compound, it.Value());
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(compound));
}

/// Thickens a shell or a face into a solid by `offset` along its normals
/// (`BRepOffsetAPI_MakeThickSolid::MakeThickSolidBySimple`: flat walls along free edges), with
/// history (see `cadrs_history`).
inline std::unique_ptr<TopoDS_Shape> cadrs_thicken_h(const TopoDS_Shape &shape, double offset,
                                                    std::vector<int32_t> &hist) {
  BRepOffsetAPI_MakeThickSolid make;
  make.MakeThickSolidBySimple(shape, offset);
  if (!make.IsDone()) {
    throw std::runtime_error("thickening failed");
  }
  const TopoDS_Shape result = make.Shape();
  cadrs_history(make, {&shape}, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// `BRepTools_History` with the `Generated` / `Modified` / `IsDeleted` interface `cadrs_history`
/// reads.
struct cadrs_tools_history {
  Handle(BRepTools_History) h;
  const TopTools_ListOfShape &Generated(const TopoDS_Shape &s) { return h->Generated(s); }
  const TopTools_ListOfShape &Modified(const TopoDS_Shape &s) { return h->Modified(s); }
  bool IsDeleted(const TopoDS_Shape &s) { return h->IsRemoved(s); }
};

/// [`cadrs_unify`] with history: faces merged into one are listed as modified into it.
inline std::unique_ptr<TopoDS_Shape> cadrs_unify_h(const TopoDS_Shape &shape, std::vector<int32_t> &hist) {
  ShapeUpgrade_UnifySameDomain unify(shape, Standard_True, Standard_True, Standard_True);
  unify.AllowInternalEdges(Standard_False);
  unify.Build();
  const TopoDS_Shape result = unify.Shape();
  cadrs_tools_history adapter{unify.History()};
  cadrs_history(adapter, {&shape}, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// Two histories one after the other (`first`, then `second` on its result), with the
/// `Generated` / `Modified` / `IsDeleted` interface `cadrs_history` reads.
template <typename First>
struct cadrs_composed_history {
  First &first;
  Handle(BRepTools_History) second;
  TopTools_ListOfShape out;

  /// The images of `list` through `second` (a shape it left alone is its own image).
  void through_second(const TopTools_ListOfShape &list) {
    out.Clear();
    for (TopTools_ListOfShape::Iterator it(list); it.More(); it.Next()) {
      const TopoDS_Shape &x = it.Value();
      if (second->IsRemoved(x)) {
        continue;
      }
      const TopTools_ListOfShape &m = second->Modified(x);
      if (m.IsEmpty()) {
        out.Append(x);
      } else {
        for (TopTools_ListOfShape::Iterator jt(m); jt.More(); jt.Next()) {
          out.Append(jt.Value());
        }
      }
    }
  }
  const TopTools_ListOfShape &Generated(const TopoDS_Shape &s) {
    TopTools_ListOfShape g = first.Generated(s);
    through_second(g);
    return out;
  }
  const TopTools_ListOfShape &Modified(const TopoDS_Shape &s) {
    TopTools_ListOfShape m = first.Modified(s);
    if (m.IsEmpty() && !first.IsDeleted(s)) {
      m.Append(s);
    }
    through_second(m);
    return out;
  }
  bool IsDeleted(const TopoDS_Shape &s) {
    if (first.IsDeleted(s)) {
      return true;
    }
    Modified(s);
    return out.IsEmpty();
  }
};

/// Fuses `a` and `b`, then merges each face of `a` with a face of `b` it meets on the same
/// surface (the seam where the bodies met goes; faces of one input stay apart), with history
/// (inputs: `a`, then `b`).
inline std::unique_ptr<TopoDS_Shape> cadrs_fuse_clean_h(const TopoDS_Shape &a, const TopoDS_Shape &b,
                                                       std::vector<int32_t> &hist) {
  BRepAlgoAPI_Fuse fuse(a, b);
  if (!fuse.IsDone() || fuse.HasErrors()) {
    throw std::runtime_error("boolean operation failed");
  }
  const TopoDS_Shape fused = fuse.Shape();
  // Which input each face of the fused shape came from (1: a, 2: b, 3: both).
  TopTools_DataMapOfShapeInteger owner;
  const TopoDS_Shape *inputs[2] = {&a, &b};
  for (int k = 0; k < 2; ++k) {
    for (TopExp_Explorer ex(*inputs[k], TopAbs_FACE); ex.More(); ex.Next()) {
      TopTools_ListOfShape images;
      if (fuse.IsDeleted(ex.Current())) {
        continue;
      }
      if (fuse.Modified(ex.Current()).IsEmpty()) {
        images.Append(ex.Current());
      } else {
        images = fuse.Modified(ex.Current());
      }
      for (TopTools_ListOfShape::Iterator it(images); it.More(); it.Next()) {
        const int bit = 1 << k;
        if (owner.IsBound(it.Value())) {
          owner.ChangeFind(it.Value()) |= bit;
        } else {
          owner.Bind(it.Value(), bit);
        }
      }
    }
  }
  // Keep every edge between two faces of the same input.
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(fused, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  ShapeUpgrade_UnifySameDomain unify(fused, Standard_True, Standard_True, Standard_True);
  unify.AllowInternalEdges(Standard_False);
  for (Standard_Integer i = 1; i <= edge_faces.Extent(); ++i) {
    const TopTools_ListOfShape &faces = edge_faces(i);
    int mask = 3;
    int n = 0;
    for (TopTools_ListOfShape::Iterator it(faces); it.More(); it.Next()) {
      mask &= owner.IsBound(it.Value()) ? owner.Find(it.Value()) : 0;
      ++n;
    }
    if (n >= 2 && mask != 0) {
      unify.KeepShape(edge_faces.FindKey(i));
    }
  }
  unify.Build();
  const TopoDS_Shape result = unify.Shape();
  cadrs_composed_history<BRepAlgoAPI_Fuse> composed{fuse, unify.History(), {}};
  cadrs_history(composed, {&a, &b}, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// Where the line through `o` along `d` crosses the faces of `shape`: per crossing, the face's
/// index (MapShapes order), the line parameter (the distance along `d` for a unit `d`) and the
/// point (5 numbers).
inline void cadrs_ray_hits(const TopoDS_Shape &shape, double ox, double oy, double oz, double dx, double dy,
                           double dz, std::vector<double> &out) {
  TopTools_IndexedMapOfShape faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  BRepIntCurveSurface_Inter inter;
  inter.Init(shape, gp_Lin(gp_Pnt(ox, oy, oz), gp_Dir(dx, dy, dz)), 1e-7);
  for (; inter.More(); inter.Next()) {
    const gp_Pnt p = inter.Pnt();
    out.push_back((double)(faces.FindIndex(inter.Face()) - 1));
    out.push_back(inter.W());
    out.push_back(p.X());
    out.push_back(p.Y());
    out.push_back(p.Z());
  }
}

/// A tight axis-aligned bounding box from the exact geometry: min xyz, then max xyz.
inline void cadrs_bbox(const TopoDS_Shape &shape, std::vector<double> &out) {
  Bnd_Box box;
  BRepBndLib::AddOptimal(shape, box, Standard_False, Standard_False);
  if (box.IsVoid()) {
    throw std::runtime_error("the shape is empty");
  }
  double x0, y0, z0, x1, y1, z1;
  box.Get(x0, y0, z0, x1, y1, z1);
  const double v[6] = {x0, y0, z0, x1, y1, z1};
  out.insert(out.end(), v, v + 6);
}

// ---------------------------------------------------------------------------------------------
// Axes of curved faces and circular edges (cadrs P3.4, revolve axes)

#include <gp_Cylinder.hxx>
#include <gp_Cone.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>

/// Per face (MapShapes order), 8 values: a flag (1 when the face has an axis: a cylinder, cone,
/// sphere, torus or surface of revolution; else 0), the axis's origin xyz and unit direction
/// xyz, and the radius (the cylinder's, the cone's reference radius, the torus's major radius,
/// the sphere's; 0 for a surface of revolution).
inline void cadrs_face_axes(const TopoDS_Shape &shape, std::vector<double> &out) {
  TopTools_IndexedMapOfShape faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  for (Standard_Integer i = 1; i <= faces.Extent(); ++i) {
    double v[8] = {0};
    try {
      BRepAdaptor_Surface s(TopoDS::Face(faces(i)));
      gp_Ax1 ax;
      bool has = true;
      double r = 0.0;
      switch (s.GetType()) {
      case GeomAbs_Cylinder:
        ax = s.Cylinder().Axis();
        r = s.Cylinder().Radius();
        break;
      case GeomAbs_Cone:
        ax = s.Cone().Axis();
        r = s.Cone().RefRadius();
        break;
      case GeomAbs_Sphere:
        ax = s.Sphere().Position().Axis();
        r = s.Sphere().Radius();
        break;
      case GeomAbs_Torus:
        ax = s.Torus().Axis();
        r = s.Torus().MajorRadius();
        break;
      case GeomAbs_SurfaceOfRevolution:
        ax = s.AxeOfRevolution();
        break;
      default:
        has = false;
      }
      if (has) {
        const gp_Pnt o = ax.Location();
        const gp_Dir d = ax.Direction();
        const double w[8] = {1.0, o.X(), o.Y(), o.Z(), d.X(), d.Y(), d.Z(), r};
        std::copy(w, w + 8, v);
      }
    } catch (const Standard_Failure &) {
      // No axis for this face.
    }
    out.insert(out.end(), v, v + 8);
  }
}

/// Per edge (MapShapes order), 8 values: a flag (1 for a circle or an arc of one, else 0), its
/// center xyz, the unit normal of its plane xyz (the circle's own axis), and its radius.
inline void cadrs_edge_circles(const TopoDS_Shape &shape, std::vector<double> &out) {
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(shape, TopAbs_EDGE, edges);
  for (Standard_Integer i = 1; i <= edges.Extent(); ++i) {
    double v[8] = {0};
    const TopoDS_Edge &edge = TopoDS::Edge(edges(i));
    try {
      if (!BRep_Tool::Degenerated(edge)) {
        BRepAdaptor_Curve c(edge);
        if (c.GetType() == GeomAbs_Circle) {
          const gp_Circ circ = c.Circle();
          const gp_Pnt o = circ.Location();
          const gp_Dir d = circ.Axis().Direction();
          const double w[8] = {1.0, o.X(), o.Y(), o.Z(), d.X(), d.Y(), d.Z(), circ.Radius()};
          std::copy(w, w + 8, v);
        }
      }
    } catch (const Standard_Failure &) {
      // Not a circle we can read.
    }
    out.insert(out.end(), v, v + 8);
  }
}

// ---------------------------------------------------------------------------------------------
// cadrs P3.6: fillets with a radius per edge (constant or varying), face normals along edges,
// validity checks

/// Fillet of `edges` with a radius per edge: `counts[i]` (t, r) pairs for edge i in `data`, `t`
/// in [0, 1] along the edge's parameter range. One pair gives the edge a constant radius; more
/// give it a radius that varies through them (`SetRadius(UandR, IC, IinC)`). OCCT continues a
/// fillet along tangent-connected edges (its contours); an edge already in a contour is not
/// added again, but its radius is still set.
inline std::unique_ptr<TopoDS_Shape> cadrs_fillet_var_h(const TopoDS_Shape &shape, const TopTools_ListOfShape &edges,
                                                       rust::Slice<const int32_t> counts,
                                                       rust::Slice<const double> data, std::vector<int32_t> &hist) {
  BRepFilletAPI_MakeFillet make(shape);
  std::vector<TopoDS_Edge> list;
  for (TopTools_ListOfShape::Iterator it(edges); it.More(); it.Next()) {
    list.push_back(TopoDS::Edge(it.Value()));
  }
  if (list.size() != counts.size()) {
    throw std::runtime_error("fillet: a radius is needed for each edge");
  }
  size_t at = 0;
  std::vector<size_t> starts;
  for (size_t i = 0; i < list.size(); ++i) {
    starts.push_back(at);
    if (counts[i] < 1) {
      throw std::runtime_error("fillet: an edge has no radius");
    }
    at += 2 * (size_t)counts[i];
  }
  if (at > data.size()) {
    throw std::runtime_error("fillet: missing radius data");
  }
  // Add every edge first (with its first radius), then set each edge's own radii.
  for (size_t i = 0; i < list.size(); ++i) {
    if (make.Contour(list[i]) == 0) {
      make.Add(data[starts[i] + 1], list[i]);
    }
  }
  for (size_t i = 0; i < list.size(); ++i) {
    const Standard_Integer ic = make.Contour(list[i]);
    if (ic == 0) {
      throw std::runtime_error("fillet: an edge could not be added");
    }
    Standard_Integer iinc = 0;
    for (Standard_Integer k = 1; k <= make.NbEdges(ic); ++k) {
      if (make.Edge(ic, k).IsSame(list[i])) {
        iinc = k;
      }
    }
    if (iinc == 0) {
      throw std::runtime_error("fillet: an edge is not in its contour");
    }
    const size_t n = (size_t)counts[i];
    const double *d = data.data() + starts[i];
    if (n == 1) {
      make.SetRadius(d[1], ic, iinc);
      continue;
    }
    double f = 0.0, l = 0.0;
    BRep_Tool::Range(list[i], f, l);
    TColgp_Array1OfPnt2d uandr(1, (Standard_Integer)n);
    for (size_t k = 0; k < n; ++k) {
      uandr.SetValue((Standard_Integer)k + 1, gp_Pnt2d(f + d[2 * k] * (l - f), d[2 * k + 1]));
    }
    make.SetRadius(uandr, ic, iinc);
  }
  make.Build();
  if (!make.IsDone()) {
    throw std::runtime_error("fillet failed");
  }
  const TopoDS_Shape result = make.Shape();
  cadrs_history(make, {&shape}, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// Along edge `index` (MapShapes order, 0-based) at `samples` evenly spaced parameters: 10 values
/// each, t in [0, 1], the point xyz, and the outward unit normal (xyz) of each of the two faces
/// around the edge, in the order `cadrs_edge_faces` lists them (zeros where a normal can't be
/// found or there is no second face).
inline void cadrs_edge_normals(const TopoDS_Shape &shape, int32_t index, int32_t samples, std::vector<double> &out) {
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(shape, TopAbs_EDGE, edges);
  if (index < 0 || index >= edges.Extent() || samples < 2) {
    throw std::runtime_error("edge normals: no such edge");
  }
  const TopoDS_Edge &edge = TopoDS::Edge(edges(index + 1));
  TopTools_IndexedDataMapOfShapeListOfShape around;
  TopExp::MapShapesAndUniqueAncestors(shape, TopAbs_EDGE, TopAbs_FACE, around);
  std::vector<TopoDS_Face> faces;
  const Standard_Integer j = around.FindIndex(edge);
  if (j > 0) {
    for (TopTools_ListOfShape::Iterator it(around(j)); it.More(); it.Next()) {
      faces.push_back(TopoDS::Face(it.Value()));
    }
  }
  double f = 0.0, l = 0.0;
  BRep_Tool::Range(edge, f, l);
  BRepAdaptor_Curve curve(edge);
  for (int32_t s = 0; s < samples; ++s) {
    const double t = (double)s / (double)(samples - 1);
    const double u = f + t * (l - f);
    const gp_Pnt p = curve.Value(u);
    double v[10] = {t, p.X(), p.Y(), p.Z(), 0, 0, 0, 0, 0, 0};
    for (size_t k = 0; k < faces.size() && k < 2; ++k) {
      double pf = 0.0, pl = 0.0;
      Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, faces[k], pf, pl);
      if (pc.IsNull()) {
        continue;
      }
      const gp_Pnt2d uv = pc->Value(u);
      BRepAdaptor_Surface surf(faces[k]);
      BRepLProp_SLProps props(surf, uv.X(), uv.Y(), 1, 1e-9);
      if (!props.IsNormalDefined()) {
        continue;
      }
      gp_Dir n = props.Normal();
      if (faces[k].Orientation() == TopAbs_REVERSED) {
        n.Reverse();
      }
      v[4 + 3 * k] = n.X();
      v[5 + 3 * k] = n.Y();
      v[6 + 3 * k] = n.Z();
    }
    out.insert(out.end(), v, v + 10);
  }
}

/// True if `shape` passes OCCT's topology and geometry checks (`BRepCheck_Analyzer`).
inline bool cadrs_is_valid(const TopoDS_Shape &shape) {
  BRepCheck_Analyzer check(shape);
  return check.IsValid();
}

// ---------------------------------------------------------------------------------------------
// Sweeps, lofts, splits and offset curves (cadrs P3.7)

#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomConvert_ApproxCurve.hxx>
#include <GeomFill_Trihedron.hxx>
#include <Precision.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_OffsetCurve.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_HArray1OfBoolean.hxx>
#include <TColStd_HArray1OfReal.hxx>
#include <TColgp_Array1OfVec.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

/// A curve at a constant distance `offset` from an ellipse (about `normal`, major axis along
/// `x_dir`, `major` >= `minor`): positive offsets lie outside it. OCCT's exact offset curve,
/// approximated by a C2 B-spline within 1e-8 mm (offset curves are fragile in booleans). The
/// whole curve if `full`, else the arc from the point `p1` to `p2` of the offset curve,
/// counter-clockwise seen from `normal` (the points become the edge's vertices).
inline std::unique_ptr<TopoDS_Edge> cadrs_edge_offset_ellipse(double cx, double cy, double cz, double nx,
                                                             double ny, double nz, double xx, double xy, double xz,
                                                             double major, double minor, double offset, bool full,
                                                             double ax, double ay, double az, double bx, double by,
                                                             double bz) {
  const gp_Dir n(nx, ny, nz);
  Handle(Geom_Ellipse) base = new Geom_Ellipse(gp_Elips(gp_Ax2(gp_Pnt(cx, cy, cz), n, gp_Dir(xx, xy, xz)), major, minor));
  if (offset < -minor * minor / major + 1e-9) {
    throw std::runtime_error("the offset is larger than the ellipse's smallest radius of curvature");
  }
  Handle(Geom_Curve) basis = base;
  if (!full) {
    Handle(Geom_OffsetCurve) whole = new Geom_OffsetCurve(base, offset, n);
    GeomAPI_ProjectPointOnCurve pa(gp_Pnt(ax, ay, az), whole);
    GeomAPI_ProjectPointOnCurve pb(gp_Pnt(bx, by, bz), whole);
    if (pa.NbPoints() == 0 || pb.NbPoints() == 0) {
      throw std::runtime_error("the arc's ends are not on the offset ellipse");
    }
    double u0 = pa.LowerDistanceParameter();
    double u1 = pb.LowerDistanceParameter();
    while (u1 <= u0 + 1e-12) {
      u1 += 2.0 * 3.14159265358979323846;
    }
    basis = new Geom_TrimmedCurve(base, u0, u1);
  }
  Handle(Geom_OffsetCurve) curve = new Geom_OffsetCurve(basis, offset, n);
  GeomConvert_ApproxCurve approx(curve, 1e-8, GeomAbs_C2, 400, 9);
  if (!approx.HasResult()) {
    throw std::runtime_error("the offset ellipse could not be approximated");
  }
  Handle(Geom_BSplineCurve) bs = approx.Curve();
  if (full) {
    BRepBuilderAPI_MakeEdge make(bs);
    return cadrs_edge_from(make);
  }
  BRepBuilderAPI_MakeEdge make(bs, gp_Pnt(ax, ay, az), gp_Pnt(bx, by, bz));
  return cadrs_edge_from(make);
}

/// The edge split at the point of it nearest `p`: a compound of the piece before the point and
/// the piece after it (along the curve's parameter). A closed periodic edge (a whole circle)
/// gives one edge that starts and ends at the point. A point at an end gives the edge itself.
inline std::unique_ptr<TopoDS_Shape> cadrs_edge_split(const TopoDS_Edge &edge, double px, double py, double pz) {
  double f = 0.0, l = 0.0;
  Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
  if (curve.IsNull()) {
    throw std::runtime_error("the edge has no 3D curve");
  }
  GeomAPI_ProjectPointOnCurve proj(gp_Pnt(px, py, pz), curve, f, l);
  if (proj.NbPoints() == 0) {
    throw std::runtime_error("the point does not project onto the edge");
  }
  const double u = proj.LowerDistanceParameter();
  BRep_Builder b;
  TopoDS_Compound out;
  b.MakeCompound(out);
  const double span = l - f;
  const bool closed = curve->Value(f).Distance(curve->Value(l)) < Precision::Confusion();
  if (closed && curve->IsPeriodic()) {
    BRepBuilderAPI_MakeEdge make(curve, u, u + curve->Period());
    b.Add(out, *cadrs_edge_from(make));
  } else if (u - f < 1e-9 * span || l - u < 1e-9 * span) {
    b.Add(out, edge);
  } else {
    BRepBuilderAPI_MakeEdge first(curve, f, u);
    BRepBuilderAPI_MakeEdge second(curve, u, l);
    b.Add(out, *cadrs_edge_from(first));
    b.Add(out, *cadrs_edge_from(second));
  }
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(out));
}

/// The same edge run the other way.
inline std::unique_ptr<TopoDS_Edge> cadrs_edge_reversed(const TopoDS_Edge &edge) {
  return std::unique_ptr<TopoDS_Edge>(new TopoDS_Edge(TopoDS::Edge(edge.Reversed())));
}

/// `n + 1` points evenly spaced by length along edge `index` (MapShapes order) of `shape`, in
/// the direction of the edge's curve parameter.
inline void cadrs_edge_samples(const TopoDS_Shape &shape, int32_t index, int32_t n, std::vector<double> &out) {
  TopTools_IndexedMapOfShape edges;
  TopExp::MapShapes(shape, TopAbs_EDGE, edges);
  if (index < 0 || index >= edges.Extent() || n < 1) {
    throw std::runtime_error("no such edge");
  }
  BRepAdaptor_Curve curve(TopoDS::Edge(edges(index + 1)));
  GCPnts_UniformAbscissa sampler(curve, n + 1, curve.FirstParameter(), curve.LastParameter());
  if (!sampler.IsDone()) {
    throw std::runtime_error("sampling the edge failed");
  }
  for (Standard_Integer i = 1; i <= sampler.NbPoints(); ++i) {
    gp_Pnt p = curve.Value(sampler.Parameter(i));
    out.push_back(p.X());
    out.push_back(p.Y());
    out.push_back(p.Z());
  }
}

/// A vertex at a point (a loft's point section).
inline std::unique_ptr<TopoDS_Shape> cadrs_vertex(double x, double y, double z) {
  BRepBuilderAPI_MakeVertex make(gp_Pnt(x, y, z));
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(make.Shape()));
}

/// Sweeps `profile` (a face, a wire, ...) along `spine` (`BRepOffsetAPI_MakePipe`), with
/// history. `mode`: 0 corrected Frenet (the profile keeps its angle to the path), 1 fixed (the
/// profile keeps its orientation in space), 2 Frenet, 3 discrete trihedron.
inline std::unique_ptr<TopoDS_Shape> cadrs_pipe_h(const TopoDS_Wire &spine, const TopoDS_Shape &profile, int32_t mode,
                                                 std::vector<int32_t> &hist) {
  GeomFill_Trihedron tri = GeomFill_IsCorrectedFrenet;
  switch (mode) {
  case 1:
    tri = GeomFill_IsFixed;
    break;
  case 2:
    tri = GeomFill_IsFrenet;
    break;
  case 3:
    tri = GeomFill_IsDiscreteTrihedron;
    break;
  default:
    break;
  }
  BRepOffsetAPI_MakePipe make(spine, profile, tri, Standard_False);
  make.Build();
  if (!make.IsDone()) {
    throw std::runtime_error("sweep failed");
  }
  const TopoDS_Shape result = make.Shape();
  const TopoDS_Shape first = make.FirstShape();
  const TopoDS_Shape last = make.LastShape();
  cadrs_history(make, {&profile}, result, &first, &last, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// Sweeps the wire `profile` along `spine` with `BRepOffsetAPI_MakePipeShell`, with history.
/// `mode`: 0 corrected Frenet, 2 Frenet, 3 discrete trihedron, 4 a fixed binormal direction
/// (`bx`, `by`, `bz`: the profile's plane keeps containing it). `solid` closes the ends of a
/// closed profile.
inline std::unique_ptr<TopoDS_Shape> cadrs_pipe_shell_h(const TopoDS_Wire &spine, const TopoDS_Wire &profile,
                                                       int32_t mode, double bx, double by, double bz, bool solid,
                                                       std::vector<int32_t> &hist) {
  BRepOffsetAPI_MakePipeShell make(spine);
  switch (mode) {
  case 2:
    make.SetMode(Standard_True);
    break;
  case 3:
    make.SetDiscreteMode();
    break;
  case 4:
    make.SetMode(gp_Dir(bx, by, bz));
    break;
  default:
    make.SetMode(Standard_False);
    break;
  }
  make.Add(profile, Standard_False, Standard_False);
  make.Build();
  if (!make.IsDone()) {
    throw std::runtime_error("sweep failed");
  }
  if (solid && !make.MakeSolid()) {
    throw std::runtime_error("the sweep could not be closed into a solid");
  }
  const TopoDS_Shape result = make.Shape();
  const TopoDS_Shape first = make.FirstShape();
  const TopoDS_Shape last = make.LastShape();
  cadrs_history(make, {&profile}, result, &first, &last, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// A loft through `sections` (wires, and vertices at the ends) with
/// `BRepOffsetAPI_ThruSections`, with history (each section edge's generated faces; the first
/// and last section's faces). `ruled` joins neighbouring sections with ruled surfaces;
/// `max_degree` > 0 limits the degree of the loft surface across the sections.
inline std::unique_ptr<TopoDS_Shape> cadrs_thru_sections_h(const TopTools_ListOfShape &sections, bool solid, bool ruled,
                                                          bool smoothing, int32_t max_degree,
                                                          std::vector<int32_t> &hist) {
  BRepOffsetAPI_ThruSections make(solid, ruled, 1e-6);
  make.CheckCompatibility(Standard_True);
  make.SetSmoothing(smoothing);
  if (max_degree > 0) {
    make.SetMaxDegree(max_degree);
  }
  std::vector<TopoDS_Shape> inputs;
  for (TopTools_ListOfShape::Iterator it(sections); it.More(); it.Next()) {
    inputs.push_back(it.Value());
  }
  for (const TopoDS_Shape &s : inputs) {
    if (s.ShapeType() == TopAbs_VERTEX) {
      make.AddVertex(TopoDS::Vertex(s));
    } else if (s.ShapeType() == TopAbs_WIRE) {
      make.AddWire(TopoDS::Wire(s));
    } else {
      throw std::runtime_error("a loft section must be a wire or a vertex");
    }
  }
  make.Build();
  if (!make.IsDone()) {
    throw std::runtime_error("loft failed");
  }
  const TopoDS_Shape result = make.Shape();
  const TopoDS_Shape first = make.FirstShape();
  const TopoDS_Shape last = make.LastShape();
  std::vector<const TopoDS_Shape *> ptrs;
  for (const TopoDS_Shape &s : inputs) {
    ptrs.push_back(&s);
  }
  cadrs_history(make, ptrs, result, &first, &last, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// The derivative across the sections at one end of a loft ([`cadrs_loft_solid`]): `spec` is
/// `[mode, x, y, z, cx, cy, cz, scale]`. Mode 0: none (the loft's curvature is zero there);
/// 1: the vector (x, y, z) at every point; 2: in the plane with normal (x, y, z), away from the
/// point c (or towards it for a negative scale), `scale` long.
static bool cadrs_loft_derivative(rust::Slice<const double> spec, const gp_Pnt &pole, gp_Vec &out) {
  if (spec.size() < 8 || spec[0] < 0.5) {
    return false;
  }
  if (spec[0] < 1.5) {
    out = gp_Vec(spec[1], spec[2], spec[3]);
    return true;
  }
  const gp_Vec n(spec[1], spec[2], spec[3]);
  gp_Vec r(gp_Pnt(spec[4], spec[5], spec[6]), pole);
  r -= n * (r.Dot(n) / n.SquareMagnitude());
  if (r.Magnitude() < 1e-12) {
    out = gp_Vec(0, 0, 0);
  } else {
    out = r.Normalized() * spec[7];
  }
  return true;
}

/// A loft built from sample points (cadrs P3.7: lofts with start and end conditions, which
/// `BRepOffsetAPI_ThruSections` has no way to take). `points` holds `patches` × `k` sections ×
/// `n` points (x, y, z): patch p's samples along its piece of section i. Each section piece is
/// interpolated by a cubic B-spline through its samples (all with the same parameters, so they
/// share one knot vector; `periodic` for a single closed smooth piece, whose `n` samples then
/// don't repeat the first), and each column of poles by a cubic B-spline through the `k`
/// sections at `vparams`, with the derivatives `start` and `end` (see
/// [`cadrs_loft_derivative`]) at the first and last section. One face per patch; `solid` adds
/// planar caps from the first and last section and sews everything into a solid, else the faces
/// are sewn into a shell.
inline std::unique_ptr<TopoDS_Shape> cadrs_loft_solid(rust::Slice<const double> points, int32_t patches, int32_t k,
                                                     int32_t n, bool periodic, rust::Slice<const double> vparams,
                                                     rust::Slice<const double> start,
                                                     rust::Slice<const double> end, bool solid) {
  if (patches < 1 || k < 2 || n < 2 || (int)vparams.size() != k ||
      (int)points.size() != patches * k * n * 3 || (periodic && patches != 1)) {
    throw std::runtime_error("bad loft samples");
  }
  BRepBuilderAPI_Sewing sew(1e-6);
  std::vector<TopoDS_Edge> first_edges, last_edges;
  for (int p = 0; p < patches; ++p) {
    // Along each section.
    const int m = periodic ? n + 1 : n;
    Handle(TColStd_HArray1OfReal) uparams = new TColStd_HArray1OfReal(1, m);
    for (int j = 0; j < m; ++j) {
      uparams->SetValue(j + 1, (double)j / (double)(periodic ? n : n - 1));
    }
    std::vector<Handle(Geom_BSplineCurve)> sections;
    for (int i = 0; i < k; ++i) {
      Handle(TColgp_HArray1OfPnt) pts = new TColgp_HArray1OfPnt(1, n);
      for (int j = 0; j < n; ++j) {
        const size_t o = (((size_t)p * k + i) * n + j) * 3;
        pts->SetValue(j + 1, gp_Pnt(points[o], points[o + 1], points[o + 2]));
      }
      GeomAPI_Interpolate interp(pts, uparams, periodic, 1e-12);
      interp.Perform();
      if (!interp.IsDone()) {
        throw std::runtime_error("a loft section could not be interpolated");
      }
      sections.push_back(interp.Curve());
    }
    const Handle(Geom_BSplineCurve) &c0 = sections[0];
    const int nu = c0->NbPoles();
    for (const Handle(Geom_BSplineCurve) &c : sections) {
      if (c->NbPoles() != nu || c->Degree() != c0->Degree()) {
        throw std::runtime_error("loft sections are not compatible");
      }
    }
    // Across the sections, one pole column at a time.
    Handle(TColStd_HArray1OfReal) vp = new TColStd_HArray1OfReal(1, k);
    for (int i = 0; i < k; ++i) {
      vp->SetValue(i + 1, vparams[i]);
    }
    std::vector<Handle(Geom_BSplineCurve)> columns;
    for (int j = 1; j <= nu; ++j) {
      Handle(TColgp_HArray1OfPnt) col = new TColgp_HArray1OfPnt(1, k);
      for (int i = 0; i < k; ++i) {
        col->SetValue(i + 1, sections[i]->Pole(j));
      }
      GeomAPI_Interpolate interp(col, vp, Standard_False, 1e-12);
      gp_Vec d0, d1;
      const bool has0 = cadrs_loft_derivative(start, col->Value(1), d0);
      const bool has1 = cadrs_loft_derivative(end, col->Value(k), d1);
      if (has0 || has1) {
        TColgp_Array1OfVec tangents(1, k);
        Handle(TColStd_HArray1OfBoolean) flags = new TColStd_HArray1OfBoolean(1, k);
        for (int i = 1; i <= k; ++i) {
          tangents.SetValue(i, gp_Vec(0, 0, 0));
          flags->SetValue(i, Standard_False);
        }
        if (has0) {
          tangents.SetValue(1, d0);
          flags->SetValue(1, Standard_True);
        }
        if (has1) {
          tangents.SetValue(k, d1);
          flags->SetValue(k, Standard_True);
        }
        interp.Load(tangents, flags, Standard_False);
      }
      interp.Perform();
      if (!interp.IsDone()) {
        throw std::runtime_error("the loft could not be interpolated across its sections");
      }
      columns.push_back(interp.Curve());
    }
    const Handle(Geom_BSplineCurve) &v0 = columns[0];
    const int nv = v0->NbPoles();
    TColgp_Array2OfPnt poles(1, nu, 1, nv);
    for (int j = 1; j <= nu; ++j) {
      if (columns[j - 1]->NbPoles() != nv) {
        throw std::runtime_error("loft columns are not compatible");
      }
      for (int i = 1; i <= nv; ++i) {
        poles.SetValue(j, i, columns[j - 1]->Pole(i));
      }
    }
    TColStd_Array1OfReal uk(1, c0->NbKnots()), vk(1, v0->NbKnots());
    TColStd_Array1OfInteger um(1, c0->NbKnots()), vm(1, v0->NbKnots());
    c0->Knots(uk);
    c0->Multiplicities(um);
    v0->Knots(vk);
    v0->Multiplicities(vm);
    Handle(Geom_BSplineSurface) surface = new Geom_BSplineSurface(poles, uk, vk, um, vm, c0->Degree(), v0->Degree(),
                                                                  c0->IsPeriodic(), Standard_False);
    BRepBuilderAPI_MakeFace face(surface, 1e-7);
    if (!face.IsDone()) {
      throw std::runtime_error("the loft face could not be built");
    }
    sew.Add(face.Face());
    if (solid) {
      BRepBuilderAPI_MakeEdge e0(sections.front());
      BRepBuilderAPI_MakeEdge e1(sections.back());
      first_edges.push_back(*cadrs_edge_from(e0));
      last_edges.push_back(*cadrs_edge_from(e1));
    }
  }
  if (solid) {
    for (const std::vector<TopoDS_Edge> *edges : {&first_edges, &last_edges}) {
      BRepBuilderAPI_MakeWire wire;
      for (const TopoDS_Edge &e : *edges) {
        wire.Add(e);
      }
      if (!wire.IsDone()) {
        throw std::runtime_error("a loft end is not a closed loop");
      }
      BRepBuilderAPI_MakeFace cap(wire.Wire(), Standard_True);
      if (!cap.IsDone()) {
        throw std::runtime_error("a loft end is not planar");
      }
      sew.Add(cap.Face());
    }
  }
  sew.Perform();
  TopoDS_Shape sewn = sew.SewedShape();
  if (!solid) {
    return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(sewn));
  }
  TopExp_Explorer shells(sewn, TopAbs_SHELL);
  if (!shells.More()) {
    throw std::runtime_error("the loft's faces do not close up");
  }
  BRepBuilderAPI_MakeSolid make(TopoDS::Shell(shells.Current()));
  if (!make.IsDone()) {
    throw std::runtime_error("the loft could not be made solid");
  }
  TopoDS_Solid result = make.Solid();
  BRepLib::OrientClosedSolid(result);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}

/// Splits `shape` by `tools` (faces, shells, ...; `BRepAlgoAPI_Splitter`): the pieces share
/// the faces the tools made. History inputs: `shape`, then each tool.
inline std::unique_ptr<TopoDS_Shape> cadrs_split_h(const TopoDS_Shape &shape, const TopTools_ListOfShape &tools,
                                                  std::vector<int32_t> &hist) {
  BRepAlgoAPI_Splitter split;
  TopTools_ListOfShape args;
  args.Append(shape);
  split.SetArguments(args);
  split.SetTools(tools);
  split.Build();
  if (!split.IsDone() || split.HasErrors()) {
    throw std::runtime_error("split failed");
  }
  const TopoDS_Shape result = split.Shape();
  std::vector<TopoDS_Shape> keep;
  for (TopTools_ListOfShape::Iterator it(tools); it.More(); it.Next()) {
    keep.push_back(it.Value());
  }
  std::vector<const TopoDS_Shape *> inputs{&shape};
  for (const TopoDS_Shape &t : keep) {
    inputs.push_back(&t);
  }
  cadrs_history(split, inputs, result, nullptr, nullptr, hist);
  return std::unique_ptr<TopoDS_Shape>(new TopoDS_Shape(result));
}
