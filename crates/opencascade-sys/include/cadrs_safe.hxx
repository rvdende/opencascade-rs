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
#include <TopExp_Explorer.hxx>
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
