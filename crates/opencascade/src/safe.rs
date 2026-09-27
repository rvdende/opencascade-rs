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
