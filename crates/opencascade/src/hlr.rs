//! Hidden line removal for drawing views (cadrs branch).
//!
//! [`hlr_project`] projects a shape orthographically onto a view plane and sorts the projected
//! edges into visible and hidden sharp edges, smooth (tangent, G1) edges and outlines
//! (silhouettes), like a drawing view. It wraps `HLRBRep_Algo` / `HLRBRep_HLRToShape` (exact)
//! or `HLRBRep_PolyAlgo` / `HLRBRep_PolyHLRToShape` (on a mesh) with an `HLRAlgo_Projector`.
//!
//! # 2D frame
//!
//! The eye looks along `view_dir` (from the eye into the scene). 2D coordinates are relative to
//! `view_origin`: `x` along `view_x` (made perpendicular to `view_dir`), `y` along
//! `view_x × view_dir` (up on the drawing sheet). Everything is in model units (mm). For example
//! a front view (looking along +Y at the XZ plane) with `view_x = +X` has 2D y = +Z; a top view
//! (`view_dir = -Z`, `view_x = +X`) has 2D y = +Y.

use glam::{dvec2, DVec2, DVec3};
use opencascade_sys as ffi;

use crate::{primitives::Shape, Error};

/// Which HLR algorithm to use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HlrAlgorithm {
    /// `HLRBRep_Algo`: exact curves (lines and circles stay lines and circles). Slower.
    Exact,
    /// `HLRBRep_PolyAlgo` on a mesh with this deflection (mm): fast, but every edge comes out
    /// as straight segments. Meshes the shape in place (like `Shape::try_mesh`).
    Poly { mesh_deflection: f64 },
}

/// The class of a projected edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HlrClass {
    /// Sharp edges (`VCompound` / `HCompound`).
    Sharp,
    /// Smooth edges between tangent faces, e.g. where a fillet meets a face
    /// (`Rg1LineVCompound` / `Rg1LineHCompound`).
    Smooth,
    /// Silhouettes of curved faces, e.g. the sides of a cylinder seen side-on
    /// (`OutLineVCompound` / `OutLineHCompound`).
    Outline,
}

/// The exact 2D curve of a projected edge, where it is a line or a circle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HlrCurve {
    Line { start: DVec2, end: DVec2 },
    /// A circular arc through `start`, `mid` and `end` (or a full circle when `start == end`).
    Arc { center: DVec2, radius: f64, start: DVec2, mid: DVec2, end: DVec2, full: bool },
    /// Anything else (ellipses, splines): use the polyline.
    Other,
}

/// One projected edge, in the view's 2D frame (see the module docs).
#[derive(Debug, Clone, PartialEq)]
pub struct HlrEdge {
    pub curve: HlrCurve,
    pub start: DVec2,
    pub end: DVec2,
    /// The point halfway along the edge's parameter range.
    pub mid: DVec2,
    /// The edge as a polyline within the requested tolerance (at least two points; exactly two
    /// for lines).
    pub polyline: Vec<DVec2>,
}

impl HlrEdge {
    /// Length of the polyline (exact for lines).
    pub fn length(&self) -> f64 {
        self.polyline.windows(2).map(|w| w[0].distance(w[1])).sum()
    }
}

/// The result of [`hlr_project`]: projected edges sorted by class and visibility.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct HlrResult {
    pub visible_sharp: Vec<HlrEdge>,
    pub visible_smooth: Vec<HlrEdge>,
    pub visible_outline: Vec<HlrEdge>,
    pub hidden_sharp: Vec<HlrEdge>,
    pub hidden_smooth: Vec<HlrEdge>,
    pub hidden_outline: Vec<HlrEdge>,
}

impl HlrResult {
    /// The edges of one class and visibility.
    pub fn edges(&self, class: HlrClass, visible: bool) -> &[HlrEdge] {
        match (class, visible) {
            (HlrClass::Sharp, true) => &self.visible_sharp,
            (HlrClass::Smooth, true) => &self.visible_smooth,
            (HlrClass::Outline, true) => &self.visible_outline,
            (HlrClass::Sharp, false) => &self.hidden_sharp,
            (HlrClass::Smooth, false) => &self.hidden_smooth,
            (HlrClass::Outline, false) => &self.hidden_outline,
        }
    }

    /// Every edge with its class and visibility.
    pub fn iter(&self) -> impl Iterator<Item = (HlrClass, bool, &HlrEdge)> {
        [
            (HlrClass::Sharp, true),
            (HlrClass::Smooth, true),
            (HlrClass::Outline, true),
            (HlrClass::Sharp, false),
            (HlrClass::Smooth, false),
            (HlrClass::Outline, false),
        ]
        .into_iter()
        .flat_map(move |(c, v)| self.edges(c, v).iter().map(move |e| (c, v, e)))
    }

    /// 2D bounding box `(min, max)` of all edges, or `None` when there are none.
    pub fn bounds(&self) -> Option<(DVec2, DVec2)> {
        let mut it = self.iter().flat_map(|(_, _, e)| e.polyline.iter().copied());
        let first = it.next()?;
        Some(it.fold((first, first), |(lo, hi), p| (lo.min(p), hi.max(p))))
    }

    /// Drops hidden edges that lie entirely on visible edges (within `tol`, mm), e.g. the edges
    /// of a box's back face right behind its front face. OCCT reports those as hidden too; a
    /// drawing would show them as dashes over solid lines. Hidden edges only partly covered are
    /// kept whole. Use a `tol` at least as large as the polyline tolerance for curved edges.
    pub fn remove_hidden_behind_visible(&mut self, tol: f64) {
        let visible: Vec<[DVec2; 2]> = [&self.visible_sharp, &self.visible_smooth, &self.visible_outline]
            .into_iter()
            .flatten()
            .flat_map(|e| e.polyline.windows(2).map(|w| [w[0], w[1]]))
            .collect();
        let near = |p: DVec2| visible.iter().any(|s| segment_distance(p, s[0], s[1]) <= tol);
        let covered = |e: &HlrEdge| {
            e.polyline.iter().all(|&p| near(p))
                && e.polyline.windows(2).all(|w| [0.25, 0.5, 0.75].iter().all(|&t| near(w[0].lerp(w[1], t))))
        };
        for list in [&mut self.hidden_sharp, &mut self.hidden_smooth, &mut self.hidden_outline] {
            list.retain(|e| !covered(e));
        }
    }
}

fn segment_distance(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let len2 = ab.length_squared();
    let t = if len2 > 0.0 { ((p - a).dot(ab) / len2).clamp(0.0, 1.0) } else { 0.0 };
    p.distance(a + ab * t)
}

/// Exact hidden line removal of `shape` (see the module docs for the 2D frame). `tolerance` is
/// the chordal deviation (mm) of the polylines.
pub fn hlr_project(
    shape: &Shape,
    view_origin: DVec3,
    view_dir: DVec3,
    view_x: DVec3,
    tolerance: f64,
) -> Result<HlrResult, Error> {
    hlr_project_with(shape, view_origin, view_dir, view_x, tolerance, HlrAlgorithm::Exact)
}

/// [`hlr_project`] with a choice of algorithm.
pub fn hlr_project_with(
    shape: &Shape,
    view_origin: DVec3,
    view_dir: DVec3,
    view_x: DVec3,
    tolerance: f64,
    algorithm: HlrAlgorithm,
) -> Result<HlrResult, Error> {
    let (exact, mesh_deflection) = match algorithm {
        HlrAlgorithm::Exact => (true, 0.0),
        HlrAlgorithm::Poly { mesh_deflection } => (false, mesh_deflection),
    };
    let mut v = ffi::cadrs_safe::cadrs_new_f64_vec();
    ffi::cadrs_safe::cadrs_hlr(
        &shape.inner,
        view_origin.x,
        view_origin.y,
        view_origin.z,
        view_dir.x,
        view_dir.y,
        view_dir.z,
        view_x.x,
        view_x.y,
        view_x.z,
        exact,
        mesh_deflection,
        tolerance,
        v.pin_mut(),
    )
    .map_err(|e| Error::Occt(e.what().to_string()))?;
    Ok(parse(v.as_slice()))
}

/// Whether `pts` run along one straight segment from the first to the last point.
fn is_straight(pts: &[DVec2]) -> bool {
    let (Some(&a), Some(&b)) = (pts.first(), pts.last()) else {
        return false;
    };
    let chord = a.distance(b);
    if chord < 1e-9 {
        return false;
    }
    let tol = 1e-7 * chord.max(1.0);
    let len: f64 = pts.windows(2).map(|w| w[0].distance(w[1])).sum();
    let dir = (b - a) / chord;
    (len - chord).abs() <= tol && pts.iter().all(|p| dir.perp_dot(*p - a).abs() <= tol)
}

/// Parses the records written by `cadrs_hlr_edges` in `include/cadrs_safe.hxx`.
fn parse(s: &[f64]) -> HlrResult {
    let mut out = HlrResult::default();
    let mut i = 0;
    while i + 12 <= s.len() {
        let category = s[i] as i32;
        let kind = s[i + 1] as i32;
        let n = s[i + 2] as usize;
        let p = |k: usize| dvec2(s[i + k], s[i + k + 1]);
        let (start, end, mid, center, radius) = (p(3), p(5), p(7), p(9), s[i + 11]);
        let end_idx = (i + 12 + 2 * n).min(s.len());
        let mut polyline: Vec<DVec2> = s[i + 12..end_idx].chunks_exact(2).map(|c| dvec2(c[0], c[1])).collect();
        i = end_idx;
        let curve = match kind {
            0 => HlrCurve::Line { start, end },
            1 => {
                let scale = radius.max(1.0);
                HlrCurve::Arc { center, radius, start, mid, end, full: start.distance(end) < 1e-9 * scale }
            }
            // Curves seen edge-on (a circle whose plane contains the view direction) come out as
            // flattened ellipses: report them as lines when the polyline is straight and doesn't
            // fold back.
            _ if is_straight(&polyline) => {
                polyline = vec![start, end];
                HlrCurve::Line { start, end }
            }
            _ => HlrCurve::Other,
        };
        let edge = HlrEdge { curve, start, end, mid, polyline };
        match category {
            0 => out.visible_sharp.push(edge),
            1 => out.visible_smooth.push(edge),
            2 => out.visible_outline.push(edge),
            3 => out.hidden_sharp.push(edge),
            4 => out.hidden_smooth.push(edge),
            _ => out.hidden_outline.push(edge),
        }
    }
    out
}
