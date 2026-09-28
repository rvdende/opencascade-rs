//! Fallible versions of the modeling operations (cadrs branch).
//!
//! Unlike most of this crate's API, these never abort the process when OpenCASCADE throws:
//! an exception or a failed builder becomes [`Error::Occt`] with the exception's message. They
//! wrap `opencascade_sys::cadrs_safe`.

use cxx::UniquePtr;
use glam::{dvec3, DVec3};
use opencascade_sys as ffi;

use crate::{
    primitives::{Edge, Face, Shape, Wire},
    Error,
};

fn occt(e: cxx::Exception) -> Error {
    Error::Occt(e.what().to_string())
}

fn shape(inner: UniquePtr<ffi::topo_ds::TopoDS_Shape>) -> Result<Shape, Error> {
    if inner.is_null() {
        return Err(Error::Occt("null shape".into()));
    }
    Ok(Shape { inner })
}

fn triples(v: &cxx::CxxVector<f64>) -> Vec<DVec3> {
    let s = v.as_slice();
    s.chunks_exact(3).map(|c| dvec3(c[0], c[1], c[2])).collect()
}

fn shape_list<'a>(shapes: impl IntoIterator<Item = &'a ffi::topo_ds::TopoDS_Shape>) -> UniquePtr<ffi::top_tools::TopTools_ListOfShape> {
    let mut list = ffi::top_tools::new_list_of_shape();
    for s in shapes {
        list.pin_mut().Append(s);
    }
    list
}

/// How a chamfer is measured on an edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChamferKind {
    /// The same distance on both faces.
    Equal(f64),
    /// `d1` on the given face, `d2` on the other.
    TwoDistances(f64, f64),
    /// `distance` on the given face and `angle` (radians) from it.
    DistanceAngle(f64, f64),
}

/// A triangulated face read back after [`Shape::try_mesh`].
#[derive(Debug, Clone, Default)]
pub struct FaceTriangulation {
    pub positions: Vec<DVec3>,
    /// Unit normals pointing out of the material.
    pub normals: Vec<DVec3>,
    /// Counter-clockwise seen from outside.
    pub indices: Vec<[u32; 3]>,
}

impl Edge {
    /// A straight edge from `a` to `b`.
    pub fn try_segment(a: DVec3, b: DVec3) -> Result<Self, Error> {
        let inner = ffi::cadrs_safe::cadrs_edge_segment(a.x, a.y, a.z, b.x, b.y, b.z).map_err(occt)?;
        Ok(Self { inner })
    }

    /// A rational Bezier curve with a weight per pole (three poles make a conic: an ellipse arc
    /// for a middle weight below 1, a parabola at 1, a hyperbola above).
    pub fn try_bezier(poles: &[DVec3], weights: &[f64]) -> Result<Self, Error> {
        let flat: Vec<f64> = poles.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
        let inner = ffi::cadrs_safe::cadrs_edge_bezier(&flat, weights).map_err(occt)?;
        Ok(Self { inner })
    }

    /// A circular arc through three points.
    pub fn try_arc(a: DVec3, b: DVec3, c: DVec3) -> Result<Self, Error> {
        let inner =
            ffi::cadrs_safe::cadrs_edge_arc3(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z).map_err(occt)?;
        Ok(Self { inner })
    }

    /// A full circle, counter-clockwise seen from `normal`, starting at `center + radius·x_dir`.
    pub fn try_circle(center: DVec3, normal: DVec3, x_dir: DVec3, radius: f64) -> Result<Self, Error> {
        let (c, n, x) = (center, normal, x_dir);
        let inner = ffi::cadrs_safe::cadrs_edge_circle(c.x, c.y, c.z, n.x, n.y, n.z, x.x, x.y, x.z, radius)
            .map_err(occt)?;
        Ok(Self { inner })
    }

    /// An ellipse, counter-clockwise seen from `normal`, with its major axis along `x_dir`
    /// (`major >= minor`): `p(t) = center + major·cos t·x_dir + minor·sin t·(normal × x_dir)`.
    /// `ends` limits it to the arc from the first point counter-clockwise to the second (the
    /// points become its vertices); `None` is the whole ellipse.
    pub fn ellipse(
        center: DVec3,
        normal: DVec3,
        x_dir: DVec3,
        major: f64,
        minor: f64,
        ends: Option<(DVec3, DVec3)>,
    ) -> Result<Self, Error> {
        let (c, n, x) = (center, normal, x_dir);
        let (full, (a, b)) = match ends {
            Some(e) => (false, e),
            None => (true, (DVec3::ZERO, DVec3::ZERO)),
        };
        let inner = ffi::cadrs_safe::cadrs_edge_ellipse(
            c.x, c.y, c.z, n.x, n.y, n.z, x.x, x.y, x.z, major, minor, full, a.x, a.y, a.z, b.x, b.y, b.z,
        )
        .map_err(occt)?;
        Ok(Self { inner })
    }

    /// Points along the edge. After [`Shape::try_mesh`] they are the mesh's own points on a face
    /// of `shape` containing the edge, so the edge lines up with the triangles; otherwise the
    /// curve is sampled within `angular` radians and `deflection`.
    pub fn polyline(&self, shape: &Shape, angular: f64, deflection: f64) -> Result<Vec<DVec3>, Error> {
        let v = ffi::cadrs_safe::cadrs_edge_polyline(&self.inner, &shape.inner, angular, deflection)
            .map_err(occt)?;
        Ok(triples(&v))
    }

    /// Equal for edges that are the same edge (same geometry and location, either orientation).
    pub fn identity_hash(&self) -> u64 {
        ffi::cadrs_safe::cadrs_shape_hash(ffi::topo_ds::cast_edge_to_shape(&self.inner))
    }

    /// True if both are the same edge (either orientation).
    pub fn is_same(&self, other: &Edge) -> bool {
        ffi::cadrs_safe::cadrs_shape_is_same(
            ffi::topo_ds::cast_edge_to_shape(&self.inner),
            ffi::topo_ds::cast_edge_to_shape(&other.inner),
        )
    }
}

impl Wire {
    /// A wire of edges joined end to start.
    pub fn try_from_edges<'a>(edges: impl IntoIterator<Item = &'a Edge>) -> Result<Self, Error> {
        let mut builder = ffi::cadrs_safe::cadrs_wire_builder();
        for edge in edges {
            ffi::cadrs_safe::cadrs_wire_add(builder.pin_mut(), &edge.inner).map_err(occt)?;
        }
        let inner = ffi::cadrs_safe::cadrs_wire_build(builder.pin_mut()).map_err(occt)?;
        Ok(Self { inner })
    }
}

/// The type of surface a face lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceType {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Other,
}

impl Face {
    /// A planar face bounded by `outer` with `holes` (holes run the other way round).
    pub fn try_from_wires(outer: &Wire, holes: &[Wire]) -> Result<Self, Error> {
        let mut inner = ffi::cadrs_safe::cadrs_face_from_wire(&outer.inner).map_err(occt)?;
        for hole in holes {
            inner = ffi::cadrs_safe::cadrs_face_add_hole(&inner, &hole.inner).map_err(occt)?;
        }
        Ok(Self { inner })
    }

    /// Sweeps the face along `dir` into a solid.
    pub fn try_extrude(&self, dir: DVec3) -> Result<Shape, Error> {
        let s = ffi::topo_ds::cast_face_to_shape(&self.inner);
        shape(ffi::cadrs_safe::cadrs_prism(s, dir.x, dir.y, dir.z).map_err(occt)?)
    }

    /// Revolves the face about the axis through `origin` along `axis` by `angle` radians.
    pub fn try_revolve(&self, origin: DVec3, axis: DVec3, angle: f64) -> Result<Shape, Error> {
        let s = ffi::topo_ds::cast_face_to_shape(&self.inner);
        let (o, d) = (origin, axis);
        shape(ffi::cadrs_safe::cadrs_revol(s, o.x, o.y, o.z, d.x, d.y, d.z, angle).map_err(occt)?)
    }

    pub fn surface_type(&self) -> Result<SurfaceType, Error> {
        let t = ffi::cadrs_safe::cadrs_face_surface_type(&self.inner).map_err(occt)?;
        Ok(match t {
            0 => SurfaceType::Plane,
            1 => SurfaceType::Cylinder,
            2 => SurfaceType::Cone,
            3 => SurfaceType::Sphere,
            4 => SurfaceType::Torus,
            _ => SurfaceType::Other,
        })
    }

    /// The face's triangulation, after [`Shape::try_mesh`] meshed the shape it belongs to.
    pub fn triangulation(&self) -> Result<FaceTriangulation, Error> {
        let mut positions = ffi::cadrs_safe::cadrs_new_f64_vec();
        let mut normals = ffi::cadrs_safe::cadrs_new_f64_vec();
        let mut indices = ffi::cadrs_safe::cadrs_new_i32_vec();
        ffi::cadrs_safe::cadrs_face_triangulation(
            &self.inner,
            positions.pin_mut(),
            normals.pin_mut(),
            indices.pin_mut(),
        )
        .map_err(occt)?;
        Ok(FaceTriangulation {
            positions: triples(&positions),
            normals: triples(&normals),
            indices: indices
                .as_slice()
                .chunks_exact(3)
                .map(|t| [t[0] as u32, t[1] as u32, t[2] as u32])
                .collect(),
        })
    }

    /// Equal for faces that are the same face (either orientation).
    pub fn identity_hash(&self) -> u64 {
        ffi::cadrs_safe::cadrs_shape_hash(ffi::topo_ds::cast_face_to_shape(&self.inner))
    }

    pub fn is_same(&self, other: &Face) -> bool {
        ffi::cadrs_safe::cadrs_shape_is_same(
            ffi::topo_ds::cast_face_to_shape(&self.inner),
            ffi::topo_ds::cast_face_to_shape(&other.inner),
        )
    }
}

impl Shape {
    fn try_boolean(&self, other: &Shape, op: i32) -> Result<Shape, Error> {
        shape(ffi::cadrs_safe::cadrs_boolean(&self.inner, &other.inner, op).map_err(occt)?)
    }

    pub fn try_union(&self, other: &Shape) -> Result<Shape, Error> {
        self.try_boolean(other, 0)
    }

    pub fn try_subtract(&self, other: &Shape) -> Result<Shape, Error> {
        self.try_boolean(other, 1)
    }

    pub fn try_intersect(&self, other: &Shape) -> Result<Shape, Error> {
        self.try_boolean(other, 2)
    }

    /// Merges neighbouring faces on the same surface, and edges on the same curve.
    pub fn try_clean(&self) -> Result<Shape, Error> {
        shape(ffi::cadrs_safe::cadrs_unify(&self.inner).map_err(occt)?)
    }

    pub fn try_fillet_edges<'a>(&self, radius: f64, edges: impl IntoIterator<Item = &'a Edge>) -> Result<Shape, Error> {
        let list = shape_list(edges.into_iter().map(|e| ffi::topo_ds::cast_edge_to_shape(&e.inner)));
        shape(ffi::cadrs_safe::cadrs_fillet(&self.inner, &list, radius).map_err(occt)?)
    }

    /// Chamfers each edge; the face of each pair is the face the first distance is measured on
    /// (ignored for [`ChamferKind::Equal`]).
    pub fn try_chamfer_edges<'a>(
        &self,
        kind: ChamferKind,
        edges: impl IntoIterator<Item = (&'a Edge, &'a Face)>,
    ) -> Result<Shape, Error> {
        let pairs: Vec<(&Edge, &Face)> = edges.into_iter().collect();
        let edge_list = shape_list(pairs.iter().map(|(e, _)| ffi::topo_ds::cast_edge_to_shape(&e.inner)));
        let face_list = shape_list(pairs.iter().map(|(_, f)| ffi::topo_ds::cast_face_to_shape(&f.inner)));
        let (mode, d1, d2) = match kind {
            ChamferKind::Equal(d) => (0, d, d),
            ChamferKind::TwoDistances(a, b) => (1, a, b),
            ChamferKind::DistanceAngle(d, a) => (2, d, a),
        };
        shape(ffi::cadrs_safe::cadrs_chamfer(&self.inner, &edge_list, &face_list, mode, d1, d2).map_err(occt)?)
    }

    /// Hollows the shape, opening `faces`; a negative `offset` thickens inward.
    pub fn try_hollow<'a>(&self, offset: f64, faces: impl IntoIterator<Item = &'a Face>) -> Result<Shape, Error> {
        let list = shape_list(faces.into_iter().map(|f| ffi::topo_ds::cast_face_to_shape(&f.inner)));
        shape(ffi::cadrs_safe::cadrs_thick_solid(&self.inner, &list, offset, 1e-4).map_err(occt)?)
    }

    /// Triangulates every face (read them back with [`Face::triangulation`]).
    pub fn try_mesh(&self, deflection: f64, angular: f64) -> Result<(), Error> {
        ffi::cadrs_safe::cadrs_mesh(&self.inner, deflection, angular).map_err(occt)
    }

    /// Equal for shapes that are the same (either orientation).
    pub fn identity_hash(&self) -> u64 {
        ffi::cadrs_safe::cadrs_shape_hash(&self.inner)
    }
}

// ---------------------------------------------------------------------------------------------
// Modeling history and topology queries (cadrs P3.2, persistent naming)
//
// Faces, edges and vertices are numbered as `TopExp::MapShapes` numbers them: in explorer order,
// each sub-shape once (its first occurrence), from 0.

/// What a modeling operation did to its inputs. Every list holds indices of faces of the result.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct History {
    /// Per face of the inputs (the inputs' faces one after another): the faces it became. Empty
    /// if it was deleted; the face itself if it was kept unchanged.
    pub faces: Vec<Vec<usize>>,
    /// Per edge of the inputs: the faces generated from it (a prism's side face, a fillet).
    pub edges: Vec<Vec<usize>>,
    /// Per vertex of the inputs: the faces generated from it (a fillet's corner patch).
    pub vertices: Vec<Vec<usize>>,
    /// The faces of a sweep's start (`FirstShape`), for prisms and revolutions.
    pub first: Vec<usize>,
    /// The faces of a sweep's end (`LastShape`).
    pub last: Vec<usize>,
}

fn parse_history(v: &[i32]) -> Result<History, Error> {
    let bad = || Error::Occt("malformed history".into());
    let next = |i: &mut usize| -> Result<usize, Error> {
        let x = *v.get(*i).ok_or_else(bad)?;
        *i += 1;
        usize::try_from(x).map_err(|_| bad())
    };
    let section = |i: &mut usize| -> Result<Vec<Vec<usize>>, Error> {
        let n = next(i)?;
        (0..n)
            .map(|_| {
                let k = next(i)?;
                (0..k).map(|_| next(i)).collect()
            })
            .collect()
    };
    let mut i = 0usize;
    let faces = section(&mut i)?;
    let edges = section(&mut i)?;
    let vertices = section(&mut i)?;
    let first = section(&mut i)?.into_iter().next().unwrap_or_default();
    let last = section(&mut i)?.into_iter().next().unwrap_or_default();
    Ok(History { faces, edges, vertices, first, last })
}

fn with_history(
    f: impl FnOnce(
        std::pin::Pin<&mut cxx::CxxVector<i32>>,
    ) -> Result<UniquePtr<ffi::topo_ds::TopoDS_Shape>, cxx::Exception>,
) -> Result<(Shape, History), Error> {
    let mut hist = ffi::cadrs_safe::cadrs_new_i32_vec();
    let inner = f(hist.pin_mut()).map_err(occt)?;
    let history = parse_history(hist.as_slice())?;
    Ok((shape(inner)?, history))
}

/// The type of curve an edge lies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurveType {
    Line,
    Circle,
    Ellipse,
    /// A degenerate edge (a cone's apex, a sphere's pole).
    Degenerate,
    Other,
}

/// The exact geometry of an edge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeGeometry {
    pub start: DVec3,
    pub end: DVec3,
    /// The point halfway along its parameter range.
    pub mid: DVec3,
    /// Unit tangents, in the direction the edge runs.
    pub start_tangent: DVec3,
    pub end_tangent: DVec3,
    /// Exact length (`GCPnts_AbscissaPoint`).
    pub length: f64,
    pub curve: CurveType,
}

fn topo_counts(s: &ffi::topo_ds::TopoDS_Shape) -> Result<[usize; 3], Error> {
    let mut v = ffi::cadrs_safe::cadrs_new_i32_vec();
    ffi::cadrs_safe::cadrs_counts(s, v.pin_mut()).map_err(occt)?;
    let s = v.as_slice();
    Ok([s[0] as usize, s[1] as usize, s[2] as usize])
}

fn edges_geometry(s: &ffi::topo_ds::TopoDS_Shape) -> Result<Vec<EdgeGeometry>, Error> {
    let mut v = ffi::cadrs_safe::cadrs_new_f64_vec();
    ffi::cadrs_safe::cadrs_edges_info(s, v.pin_mut()).map_err(occt)?;
    Ok(v.as_slice()
        .chunks_exact(17)
        .map(|c| {
            let p = |i: usize| dvec3(c[i], c[i + 1], c[i + 2]);
            EdgeGeometry {
                start: p(0),
                end: p(3),
                mid: p(6),
                start_tangent: p(9),
                end_tangent: p(12),
                length: c[15],
                curve: match c[16] as i32 {
                    -1 => CurveType::Degenerate,
                    0 => CurveType::Line,
                    1 => CurveType::Circle,
                    2 => CurveType::Ellipse,
                    _ => CurveType::Other,
                },
            }
        })
        .collect())
}

/// Entries of a count followed by that many indices.
fn entries(v: &[i32]) -> Vec<Vec<usize>> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < v.len() {
        let k = v[i].max(0) as usize;
        let end = (i + 1 + k).min(v.len());
        out.push(v[i + 1..end].iter().map(|x| *x as usize).collect());
        i = end;
    }
    out
}

impl Face {
    /// [`Face::try_extrude`], with what the prism made from each part of the face.
    pub fn try_extrude_h(&self, dir: DVec3) -> Result<(Shape, History), Error> {
        let s = ffi::topo_ds::cast_face_to_shape(&self.inner);
        with_history(|h| ffi::cadrs_safe::cadrs_prism_h(s, dir.x, dir.y, dir.z, h))
    }

    /// [`Face::try_revolve`], with history.
    pub fn try_revolve_h(&self, origin: DVec3, axis: DVec3, angle: f64) -> Result<(Shape, History), Error> {
        let s = ffi::topo_ds::cast_face_to_shape(&self.inner);
        let (o, d) = (origin, axis);
        with_history(|h| ffi::cadrs_safe::cadrs_revol_h(s, o.x, o.y, o.z, d.x, d.y, d.z, angle, h))
    }

    /// The face's edges (MapShapes order), with their exact geometry.
    pub fn edges_geometry(&self) -> Result<Vec<EdgeGeometry>, Error> {
        edges_geometry(ffi::topo_ds::cast_face_to_shape(&self.inner))
    }

    /// The number of distinct faces (1), edges and vertices of the face.
    pub fn topology_counts(&self) -> Result<[usize; 3], Error> {
        topo_counts(ffi::topo_ds::cast_face_to_shape(&self.inner))
    }
}

impl Shape {
    fn try_boolean_h(&self, other: &Shape, op: i32) -> Result<(Shape, History), Error> {
        with_history(|h| ffi::cadrs_safe::cadrs_boolean_h(&self.inner, &other.inner, op, h))
    }

    /// [`Shape::try_union`] with history (this shape's faces first, then `other`'s).
    pub fn try_union_h(&self, other: &Shape) -> Result<(Shape, History), Error> {
        self.try_boolean_h(other, 0)
    }

    pub fn try_subtract_h(&self, other: &Shape) -> Result<(Shape, History), Error> {
        self.try_boolean_h(other, 1)
    }

    pub fn try_intersect_h(&self, other: &Shape) -> Result<(Shape, History), Error> {
        self.try_boolean_h(other, 2)
    }

    pub fn try_fillet_edges_h<'a>(
        &self,
        radius: f64,
        edges: impl IntoIterator<Item = &'a Edge>,
    ) -> Result<(Shape, History), Error> {
        let list = shape_list(edges.into_iter().map(|e| ffi::topo_ds::cast_edge_to_shape(&e.inner)));
        with_history(|h| ffi::cadrs_safe::cadrs_fillet_h(&self.inner, &list, radius, h))
    }

    pub fn try_chamfer_edges_h<'a>(
        &self,
        kind: ChamferKind,
        edges: impl IntoIterator<Item = (&'a Edge, &'a Face)>,
    ) -> Result<(Shape, History), Error> {
        let pairs: Vec<(&Edge, &Face)> = edges.into_iter().collect();
        let edge_list = shape_list(pairs.iter().map(|(e, _)| ffi::topo_ds::cast_edge_to_shape(&e.inner)));
        let face_list = shape_list(pairs.iter().map(|(_, f)| ffi::topo_ds::cast_face_to_shape(&f.inner)));
        let (mode, d1, d2) = match kind {
            ChamferKind::Equal(d) => (0, d, d),
            ChamferKind::TwoDistances(a, b) => (1, a, b),
            ChamferKind::DistanceAngle(d, a) => (2, d, a),
        };
        with_history(|h| {
            ffi::cadrs_safe::cadrs_chamfer_h(&self.inner, &edge_list, &face_list, mode, d1, d2, h)
        })
    }

    pub fn try_hollow_h<'a>(
        &self,
        offset: f64,
        faces: impl IntoIterator<Item = &'a Face>,
    ) -> Result<(Shape, History), Error> {
        let list = shape_list(faces.into_iter().map(|f| ffi::topo_ds::cast_face_to_shape(&f.inner)));
        with_history(|h| ffi::cadrs_safe::cadrs_thick_solid_h(&self.inner, &list, offset, 1e-4, h))
    }

    /// The number of distinct faces, edges and vertices.
    pub fn topology_counts(&self) -> Result<[usize; 3], Error> {
        topo_counts(&self.inner)
    }

    /// Every edge (MapShapes order) with its exact geometry.
    pub fn edges_geometry(&self) -> Result<Vec<EdgeGeometry>, Error> {
        edges_geometry(&self.inner)
    }

    /// Per edge (MapShapes order): the faces around it (MapShapes order).
    pub fn edge_faces(&self) -> Result<Vec<Vec<usize>>, Error> {
        let mut v = ffi::cadrs_safe::cadrs_new_i32_vec();
        ffi::cadrs_safe::cadrs_edge_faces(&self.inner, v.pin_mut()).map_err(occt)?;
        Ok(entries(v.as_slice()))
    }

    /// Per vertex (MapShapes order): its point and the edges that end at it.
    pub fn vertices_info(&self) -> Result<Vec<(DVec3, Vec<usize>)>, Error> {
        let mut pts = ffi::cadrs_safe::cadrs_new_f64_vec();
        let mut edges = ffi::cadrs_safe::cadrs_new_i32_vec();
        ffi::cadrs_safe::cadrs_vertices(&self.inner, pts.pin_mut(), edges.pin_mut()).map_err(occt)?;
        let points = triples(&pts);
        Ok(points.into_iter().zip(entries(edges.as_slice())).collect())
    }
}

// ---------------------------------------------------------------------------------------------
// Sub-shapes, compounds, thickening, rays and bounding boxes (cadrs P3.3)

/// A kind of sub-shape, for [`Shape::sub_shapes`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubKind {
    Solid = 0,
    Shell = 1,
    Face = 2,
    Wire = 3,
    Edge = 4,
}

/// Where a line crosses a face of a shape ([`Shape::ray_hits`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// The face's index (MapShapes order).
    pub face: usize,
    /// The line parameter: the distance from the origin along a unit direction.
    pub t: f64,
    pub point: DVec3,
}

impl Shape {
    /// The number of distinct sub-shapes of a kind.
    pub fn sub_count(&self, kind: SubKind) -> Result<usize, Error> {
        Ok(ffi::cadrs_safe::cadrs_sub_count(&self.inner, kind as i32).map_err(occt)?.max(0) as usize)
    }

    /// The distinct sub-shapes of a kind (MapShapes order). They share their faces, edges and
    /// vertices with this shape.
    pub fn sub_shapes(&self, kind: SubKind) -> Result<Vec<Shape>, Error> {
        (0..self.sub_count(kind)?)
            .map(|i| shape(ffi::cadrs_safe::cadrs_sub_shape(&self.inner, kind as i32, i as i32).map_err(occt)?))
            .collect()
    }

    /// A compound of the shapes (sharing their sub-shapes).
    pub fn try_compound<'a>(shapes: impl IntoIterator<Item = &'a Shape>) -> Result<Shape, Error> {
        let list = shape_list(shapes.into_iter().map(|s| &*s.inner));
        shape(ffi::cadrs_safe::cadrs_compound(&list).map_err(occt)?)
    }

    /// Thickens a shell or a face into a solid, `offset` along its normals (flat walls along its
    /// free edges), with history.
    pub fn try_thicken_h(&self, offset: f64) -> Result<(Shape, History), Error> {
        with_history(|h| ffi::cadrs_safe::cadrs_thicken_h(&self.inner, offset, h))
    }

    /// [`Shape::try_union_h`], then each face of `self` merged with a face of `other` it meets
    /// on the same surface (no seam where the two met; faces of one input stay apart).
    pub fn try_union_clean_h(&self, other: &Shape) -> Result<(Shape, History), Error> {
        with_history(|h| ffi::cadrs_safe::cadrs_fuse_clean_h(&self.inner, &other.inner, h))
    }

    /// [`Shape::try_clean`] with history: faces merged into one are modified into it.
    pub fn try_clean_h(&self) -> Result<(Shape, History), Error> {
        with_history(|h| ffi::cadrs_safe::cadrs_unify_h(&self.inner, h))
    }

    /// Every crossing of the line through `origin` along `dir` with a face, in no particular
    /// order.
    pub fn ray_hits(&self, origin: DVec3, dir: DVec3) -> Result<Vec<RayHit>, Error> {
        let mut v = ffi::cadrs_safe::cadrs_new_f64_vec();
        let (o, d) = (origin, dir);
        ffi::cadrs_safe::cadrs_ray_hits(&self.inner, o.x, o.y, o.z, d.x, d.y, d.z, v.pin_mut()).map_err(occt)?;
        Ok(v.as_slice()
            .chunks_exact(5)
            .map(|c| RayHit {
                face: c[0].max(0.0) as usize,
                t: c[1],
                point: dvec3(c[2], c[3], c[4]),
            })
            .collect())
    }

    /// A tight axis-aligned bounding box (min, max) from the exact geometry.
    pub fn bbox(&self) -> Result<(DVec3, DVec3), Error> {
        let mut v = ffi::cadrs_safe::cadrs_new_f64_vec();
        ffi::cadrs_safe::cadrs_bbox(&self.inner, v.pin_mut()).map_err(occt)?;
        let s = v.as_slice();
        Ok((dvec3(s[0], s[1], s[2]), dvec3(s[3], s[4], s[5])))
    }
}

impl Wire {
    /// Sweeps the wire along `dir` into a shell (one face per edge), with history.
    pub fn try_extrude_h(&self, dir: DVec3) -> Result<(Shape, History), Error> {
        let s = ffi::topo_ds::cast_wire_to_shape(&self.inner);
        with_history(|h| ffi::cadrs_safe::cadrs_prism_h(s, dir.x, dir.y, dir.z, h))
    }

    /// The wire's edges (MapShapes order), with their exact geometry.
    pub fn edges_geometry(&self) -> Result<Vec<EdgeGeometry>, Error> {
        edges_geometry(ffi::topo_ds::cast_wire_to_shape(&self.inner))
    }

    /// The wire as a shape.
    pub fn to_shape(&self) -> Shape {
        Shape::from_shape(ffi::topo_ds::cast_wire_to_shape(&self.inner))
    }

    /// Revolves the wire about the axis through `origin` along `axis` by `angle` radians into a
    /// shell (one face per edge), with history.
    pub fn try_revolve_h(&self, origin: DVec3, axis: DVec3, angle: f64) -> Result<(Shape, History), Error> {
        let s = ffi::topo_ds::cast_wire_to_shape(&self.inner);
        let (o, d) = (origin, axis);
        with_history(|h| ffi::cadrs_safe::cadrs_revol_h(s, o.x, o.y, o.z, d.x, d.y, d.z, angle, h))
    }
}

// ---------------------------------------------------------------------------------------------
// Axes of curved faces and circular edges (cadrs P3.4)

/// An axis in space with a radius: a cylinder's, cone's, sphere's or torus's axis, or a
/// circle's center and normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisInfo {
    pub origin: DVec3,
    /// Unit direction.
    pub dir: DVec3,
    /// The cylinder's or circle's radius, a cone's reference radius, a torus's major radius, a
    /// sphere's radius (0 for a general surface of revolution).
    pub radius: f64,
}

fn axis_list(v: &cxx::CxxVector<f64>) -> Vec<Option<AxisInfo>> {
    v.as_slice()
        .chunks_exact(8)
        .map(|c| {
            (c[0] > 0.5).then(|| AxisInfo {
                origin: dvec3(c[1], c[2], c[3]),
                dir: dvec3(c[4], c[5], c[6]),
                radius: c[7],
            })
        })
        .collect()
}

impl Shape {
    /// Revolves the shape (a face, wire, shell, ...) about the axis through `origin` along
    /// `axis` by `angle` radians, with history.
    pub fn try_revolve_h(&self, origin: DVec3, axis: DVec3, angle: f64) -> Result<(Shape, History), Error> {
        let (o, d) = (origin, axis);
        with_history(|h| ffi::cadrs_safe::cadrs_revol_h(&self.inner, o.x, o.y, o.z, d.x, d.y, d.z, angle, h))
    }

    /// Per face (MapShapes order): its axis, for cylinders, cones, spheres, tori and surfaces of
    /// revolution.
    pub fn face_axes(&self) -> Result<Vec<Option<AxisInfo>>, Error> {
        let mut v = ffi::cadrs_safe::cadrs_new_f64_vec();
        ffi::cadrs_safe::cadrs_face_axes(&self.inner, v.pin_mut()).map_err(occt)?;
        Ok(axis_list(&v))
    }

    /// Per edge (MapShapes order): its circle (center, plane normal, radius), for circles and
    /// arcs.
    pub fn edge_circles(&self) -> Result<Vec<Option<AxisInfo>>, Error> {
        let mut v = ffi::cadrs_safe::cadrs_new_f64_vec();
        ffi::cadrs_safe::cadrs_edge_circles(&self.inner, v.pin_mut()).map_err(occt)?;
        Ok(axis_list(&v))
    }

    /// Fillets `edges`, each with its own radius: one `(t, r)` pair for a constant radius,
    /// several for a radius that varies along the edge through them (`t` in [0, 1] along the
    /// edge's parameter range). With history.
    pub fn try_fillet_variable_h<'a>(
        &self,
        edges: impl IntoIterator<Item = (&'a Edge, &'a [(f64, f64)])>,
    ) -> Result<(Shape, History), Error> {
        let pairs: Vec<(&Edge, &[(f64, f64)])> = edges.into_iter().collect();
        let list = shape_list(pairs.iter().map(|(e, _)| ffi::topo_ds::cast_edge_to_shape(&e.inner)));
        let counts: Vec<i32> = pairs.iter().map(|(_, r)| r.len() as i32).collect();
        let data: Vec<f64> = pairs
            .iter()
            .flat_map(|(_, r)| r.iter().flat_map(|(t, r)| [*t, *r]))
            .collect();
        with_history(|h| ffi::cadrs_safe::cadrs_fillet_var_h(&self.inner, &list, &counts, &data, h))
    }

    /// Along edge `index` (MapShapes order), at `samples` evenly spaced parameters: the
    /// parameter `t` in [0, 1], the point, and the outward unit normals of the faces on either
    /// side (in `edge_faces` order; zero where there is none).
    pub fn edge_normals(&self, index: usize, samples: usize) -> Result<Vec<EdgeNormals>, Error> {
        let mut v = ffi::cadrs_safe::cadrs_new_f64_vec();
        ffi::cadrs_safe::cadrs_edge_normals(&self.inner, index as i32, samples as i32, v.pin_mut())
            .map_err(occt)?;
        Ok(v.as_slice()
            .chunks_exact(10)
            .map(|c| EdgeNormals {
                t: c[0],
                point: dvec3(c[1], c[2], c[3]),
                normals: [dvec3(c[4], c[5], c[6]), dvec3(c[7], c[8], c[9])],
            })
            .collect())
    }

    /// True if the shape passes OCCT's checks (`BRepCheck_Analyzer`).
    pub fn is_valid(&self) -> Result<bool, Error> {
        ffi::cadrs_safe::cadrs_is_valid(&self.inner).map_err(occt)
    }
}

/// A sample along an edge: see [`Shape::edge_normals`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeNormals {
    pub t: f64,
    pub point: DVec3,
    pub normals: [DVec3; 2],
}
