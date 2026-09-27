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
#include <Poly_Connect.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepTools_History.hxx>
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
