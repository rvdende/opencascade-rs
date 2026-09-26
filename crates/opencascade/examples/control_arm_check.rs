//! Builds the "Control Arm" exercise from the Onshape Introduction to Part Studios course and
//! checks its volume against the value the course shows (368 749.705 mm³).
//!
//! Sketch on Top (XY), mm: a Ø70 hub with a Ø35 bore and a 45×10 keyway at the origin, Ø35 / Ø20
//! eyes at x = ±107.5 (overall length 250), and webs along the external tangents between the hub
//! and each eye. The hub, right web and right eye are extruded 40 mm; the left web and left eye
//! are extruded 25 mm and added.

use glam::{dvec3, DVec3};
use opencascade::primitives::{Face, Shape, Wire};

const HUB_R: f64 = 35.0;
const BORE_R: f64 = 17.5;
const EYE_R: f64 = 17.5;
const EYE_HOLE_R: f64 = 10.0;
const EYE_X: f64 = 107.5;
const KEY_W: f64 = 45.0;
const KEY_H: f64 = 10.0;
const RIGHT_T: f64 = 40.0;
const LEFT_T: f64 = 25.0;
const EXPECTED_VOLUME: f64 = 368_749.705;

fn disk(center_x: f64, r: f64, h: f64) -> Shape {
    Shape::cylinder(dvec3(center_x, 0.0, 0.0), r, DVec3::Z, h)
}

/// The quadrilateral between the two external tangents of the hub and the eye at `side * EYE_X`.
fn web(side: f64, h: f64) -> Shape {
    // External tangents of circles (0, HUB_R) and (EYE_X, EYE_R): the tangent points lie along
    // the normal (cos t, ±sin t) with cos t = (HUB_R - EYE_R) / EYE_X.
    let cos_t = (HUB_R - EYE_R) / EYE_X;
    let sin_t = (1.0 - cos_t * cos_t).sqrt();
    let points = [
        dvec3(side * HUB_R * cos_t, HUB_R * sin_t, 0.0),
        dvec3(side * (EYE_X + EYE_R * cos_t), EYE_R * sin_t, 0.0),
        dvec3(side * (EYE_X + EYE_R * cos_t), -EYE_R * sin_t, 0.0),
        dvec3(side * HUB_R * cos_t, -HUB_R * sin_t, 0.0),
    ];
    let wire = Wire::from_ordered_points(points).expect("web outline");
    Face::from_wire(&wire).extrude(DVec3::Z * h).into()
}

fn main() {
    let keyway = Shape::box_from_corners(
        dvec3(-KEY_W / 2.0, -KEY_H / 2.0, 0.0),
        dvec3(KEY_W / 2.0, KEY_H / 2.0, RIGHT_T),
    );

    // Extrude 1 (40 mm): hub ring, right web, right eye ring.
    let right: Shape = disk(0.0, HUB_R, RIGHT_T)
        .union(&web(1.0, RIGHT_T))
        .into();
    let right: Shape = right.union(&disk(EYE_X, EYE_R, RIGHT_T)).into();
    let right: Shape = right.subtract(&disk(0.0, BORE_R, RIGHT_T)).into();
    let right: Shape = right.subtract(&keyway).into();
    let right: Shape = right.subtract(&disk(EYE_X, EYE_HOLE_R, RIGHT_T)).into();

    // Extrude 2 (25 mm, Add): left web and left eye ring, without the hub disk.
    let left: Shape = web(-1.0, LEFT_T).union(&disk(-EYE_X, EYE_R, LEFT_T)).into();
    let left: Shape = left.subtract(&disk(0.0, HUB_R, LEFT_T)).into();
    let left: Shape = left.subtract(&disk(-EYE_X, EYE_HOLE_R, LEFT_T)).into();

    let part: Shape = right.union(&left).into();
    let props = part.mass_properties();

    println!("volume        {:.3} mm³ (course: {EXPECTED_VOLUME:.3})", props.volume);
    println!("surface area  {:.3} mm²", props.surface_area);
    println!("centre of mass {:.3?}", props.center_of_mass);

    let error = (props.volume - EXPECTED_VOLUME).abs();
    assert!(error < 0.01, "volume differs from the course by {error:.4} mm³");
    println!("OK: volume matches the course to {error:.4} mm³");

    part.write_step("control_arm.step").expect("write STEP");
    println!("wrote control_arm.step");
}
