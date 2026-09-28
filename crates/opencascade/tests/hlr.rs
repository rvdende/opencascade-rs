use glam::{dvec2, dvec3, DVec3};
use opencascade::{
    hlr::{hlr_project, hlr_project_with, HlrAlgorithm, HlrCurve, HlrEdge, HlrResult},
    primitives::Shape,
};

const TOL: f64 = 0.01;

/// A 20 mm cube centred on the origin with a Ø10 through hole along Z.
fn cube_with_hole() -> Shape {
    let cube = Shape::box_centered(20.0, 20.0, 20.0);
    let hole = Shape::cylinder(dvec3(0.0, 0.0, -15.0), 5.0, DVec3::Z, 30.0);
    cube.subtract(&hole).into()
}

fn dump(name: &str, r: &HlrResult) {
    for (class, visible, e) in r.iter() {
        eprintln!("{name}: {class:?} visible={visible} {:?} len={:.3}", e.curve, e.length());
    }
}

fn is_line(e: &HlrEdge, len: f64) -> bool {
    matches!(e.curve, HlrCurve::Line { .. }) && (e.length() - len).abs() < 1e-6
}

#[test]
fn cube_with_hole_from_x_shows_hidden_hole_silhouette() {
    // Seen from +X: looking along -X, 2D x = world Y, 2D y = Y × (-X) = world Z.
    let r = hlr_project(&cube_with_hole(), DVec3::ZERO, -DVec3::X, DVec3::Y, TOL).unwrap();
    dump("from +X", &r);
    let hidden: Vec<&HlrEdge> = r.hidden_outline.iter().filter(|e| is_line(e, 20.0)).collect();
    assert_eq!(hidden.len(), 2, "hidden hole silhouettes");
    for e in &hidden {
        assert!((e.start.x.abs() - 5.0).abs() < 1e-6 && (e.end.x.abs() - 5.0).abs() < 1e-6);
        assert!((e.start.y.abs() - 10.0).abs() < 1e-6 && (e.end.y.abs() - 10.0).abs() < 1e-6);
    }
    assert!(hidden[0].start.x * hidden[1].start.x < 0.0, "one on each side");
    // Nothing of the hole is visible from outside.
    assert!(r.visible_outline.is_empty());
    assert_eq!(r.visible_sharp.iter().filter(|e| is_line(e, 20.0)).count(), 4);
    let (lo, hi) = r.bounds().unwrap();
    assert!(lo.distance(dvec2(-10.0, -10.0)) < 1e-6 && hi.distance(dvec2(10.0, 10.0)) < 1e-6);
    // The back face's edges and the hole's end circles lie right behind visible edges.
    let mut r = r;
    r.remove_hidden_behind_visible(1e-6);
    assert!(r.hidden_sharp.is_empty());
    assert_eq!(r.hidden_outline.len(), 2);
}

#[test]
fn cube_with_hole_from_top_shows_circle_and_square() {
    // Seen from +Z: looking along -Z, 2D x = world X, 2D y = X × (-Z) = world Y.
    let r = hlr_project(&cube_with_hole(), DVec3::ZERO, -DVec3::Z, DVec3::X, TOL).unwrap();
    dump("from +Z", &r);
    let lines: Vec<&HlrEdge> = r.visible_sharp.iter().filter(|e| is_line(e, 20.0)).collect();
    assert_eq!(lines.len(), 4, "the square");
    let arcs: Vec<(f64, f64)> = r
        .visible_sharp
        .iter()
        .filter_map(|e| match e.curve {
            HlrCurve::Arc { center, radius, .. } => Some((center.length(), radius)),
            _ => None,
        })
        .collect();
    assert!(!arcs.is_empty(), "the hole's circle");
    for (c, radius) in &arcs {
        assert!(*c < 1e-6 && (radius - 5.0).abs() < 1e-6);
    }
    // The visible arcs make up the whole circle (2π · 5).
    let circle_len: f64 = r
        .visible_sharp
        .iter()
        .filter(|e| matches!(e.curve, HlrCurve::Arc { .. }))
        .map(|e| e.length())
        .sum();
    assert!(circle_len > 0.99 * std::f64::consts::TAU * 5.0 && circle_len <= std::f64::consts::TAU * 5.0 * 1.001 * 2.0);
    for e in &r.visible_sharp {
        for p in &e.polyline {
            if matches!(e.curve, HlrCurve::Arc { .. }) {
                assert!((p.length() - 5.0).abs() < 1e-6);
            }
        }
    }
}

#[test]
fn cylinder_side_on_has_outlines() {
    let cyl = Shape::cylinder(DVec3::ZERO, 5.0, DVec3::Z, 20.0);
    let r = hlr_project(&cyl, DVec3::ZERO, -DVec3::X, DVec3::Y, TOL).unwrap();
    dump("cylinder", &r);
    let outlines: Vec<&HlrEdge> = r.visible_outline.iter().filter(|e| is_line(e, 20.0)).collect();
    assert_eq!(outlines.len(), 2);
    let mut xs: Vec<f64> = outlines.iter().map(|e| e.start.x).collect();
    xs.sort_by(f64::total_cmp);
    assert!((xs[0] + 5.0).abs() < 1e-6 && (xs[1] - 5.0).abs() < 1e-6);
    // The end circles, seen edge-on, project to straight lines 10 long (split where the
    // outlines touch them); their back halves are hidden right behind them.
    assert!(r.visible_sharp.iter().all(|e| matches!(e.curve, HlrCurve::Line { .. })));
    let ends: f64 = r.visible_sharp.iter().map(|e| e.length()).sum();
    assert!((ends - 20.0).abs() < 1e-6, "{ends}");
    assert_eq!(r.hidden_sharp.len(), 2);
    let mut r = r;
    r.remove_hidden_behind_visible(1e-6);
    assert!(r.hidden_sharp.is_empty());

    let poly = hlr_project_with(
        &cyl,
        DVec3::ZERO,
        -DVec3::X,
        DVec3::Y,
        TOL,
        HlrAlgorithm::Poly { mesh_deflection: 0.01 },
    )
    .unwrap();
    dump("cylinder poly", &poly);
    assert!(!poly.visible_outline.is_empty());
}

#[test]
fn view_frame_convention() {
    // A box from (0,0,0) to (10,20,30), seen from the front (looking along +Y) with x = +X:
    // 2D y = X × Y = +Z.
    let b = Shape::box_from_corners(DVec3::ZERO, dvec3(10.0, 20.0, 30.0));
    let r = hlr_project(&b, DVec3::ZERO, DVec3::Y, DVec3::X, TOL).unwrap();
    let (lo, hi) = r.bounds().unwrap();
    assert!(lo.distance(dvec2(0.0, 0.0)) < 1e-6 && hi.distance(dvec2(10.0, 30.0)) < 1e-6, "{lo} {hi}");
    // Relative to the origin.
    let r = hlr_project(&b, dvec3(5.0, 0.0, 5.0), DVec3::Y, DVec3::X, TOL).unwrap();
    let (lo, hi) = r.bounds().unwrap();
    assert!(lo.distance(dvec2(-5.0, -5.0)) < 1e-6 && hi.distance(dvec2(5.0, 25.0)) < 1e-6, "{lo} {hi}");

    // A blind hole from the top: visible from above, hidden from below.
    let blind = Shape::box_centered(20.0, 20.0, 20.0)
        .subtract(&Shape::cylinder(dvec3(0.0, 0.0, 0.0), 4.0, DVec3::Z, 20.0))
        .into();
    let arcs = |r: &[HlrEdge]| r.iter().filter(|e| matches!(e.curve, HlrCurve::Arc { .. })).count();
    let top = hlr_project(&blind, DVec3::ZERO, -DVec3::Z, DVec3::X, TOL).unwrap();
    dump("blind top", &top);
    assert!(arcs(&top.visible_sharp) > 0);
    let bottom = hlr_project(&blind, DVec3::ZERO, DVec3::Z, DVec3::X, TOL).unwrap();
    dump("blind bottom", &bottom);
    assert_eq!(arcs(&bottom.visible_sharp), 0);
    assert!(arcs(&bottom.hidden_sharp) > 0);
}

#[test]
fn filleted_box_has_smooth_edges() {
    let b = Shape::box_centered(20.0, 20.0, 20.0).fillet(2.0);
    let t = std::time::Instant::now();
    let r = hlr_project(&b, DVec3::ZERO, -DVec3::Z, DVec3::X, TOL).unwrap();
    eprintln!("filleted box HLR: {:?}", t.elapsed());
    dump("filleted", &r);
    assert!(!r.visible_smooth.is_empty());
    // The outer silhouette is still the 20 x 20 square.
    let (lo, hi) = r.bounds().unwrap();
    assert!(lo.distance(dvec2(-10.0, -10.0)) < 1e-6 && hi.distance(dvec2(10.0, 10.0)) < 1e-6, "{lo} {hi}");
}
