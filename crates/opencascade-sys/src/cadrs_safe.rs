//! Exception-safe wrappers for the operations cadrs uses (see `include/cadrs_safe.hxx`).
//!
//! Every function returns a `Result`: an OpenCASCADE exception (`Standard_Failure`) or a failed
//! builder becomes an `Err` with the exception's type and message, instead of aborting the
//! process.
pub use inner::*;

#[cxx::bridge]
mod inner {
    unsafe extern "C++" {
        include!("opencascade-sys/include/cadrs_safe.hxx");

        type TopoDS_Shape = crate::topo_ds::TopoDS_Shape;
        type TopoDS_Edge = crate::topo_ds::TopoDS_Edge;
        type TopoDS_Wire = crate::topo_ds::TopoDS_Wire;
        type TopoDS_Face = crate::topo_ds::TopoDS_Face;
        type TopTools_ListOfShape = crate::top_tools::TopTools_ListOfShape;
        type BRepBuilderAPI_MakeWire = crate::b_rep_builder_api::BRepBuilderAPI_MakeWire;

        // Edges
        pub fn cadrs_edge_segment(
            ax: f64,
            ay: f64,
            az: f64,
            bx: f64,
            by: f64,
            bz: f64,
        ) -> Result<UniquePtr<TopoDS_Edge>>;
        #[allow(clippy::too_many_arguments)]
        pub fn cadrs_edge_arc3(
            ax: f64,
            ay: f64,
            az: f64,
            bx: f64,
            by: f64,
            bz: f64,
            cx: f64,
            cy: f64,
            cz: f64,
        ) -> Result<UniquePtr<TopoDS_Edge>>;
        #[allow(clippy::too_many_arguments)]
        pub fn cadrs_edge_circle(
            cx: f64,
            cy: f64,
            cz: f64,
            nx: f64,
            ny: f64,
            nz: f64,
            xx: f64,
            xy: f64,
            xz: f64,
            radius: f64,
        ) -> Result<UniquePtr<TopoDS_Edge>>;
        #[allow(clippy::too_many_arguments)]
        pub fn cadrs_edge_ellipse(
            cx: f64,
            cy: f64,
            cz: f64,
            nx: f64,
            ny: f64,
            nz: f64,
            xx: f64,
            xy: f64,
            xz: f64,
            major: f64,
            minor: f64,
            full: bool,
            ax: f64,
            ay: f64,
            az: f64,
            bx: f64,
            by: f64,
            bz: f64,
        ) -> Result<UniquePtr<TopoDS_Edge>>;
        pub fn cadrs_edge_polyline(
            edge: &TopoDS_Edge,
            shape: &TopoDS_Shape,
            angular: f64,
            deflection: f64,
        ) -> Result<UniquePtr<CxxVector<f64>>>;

        // Wires and faces
        pub fn cadrs_wire_builder() -> UniquePtr<BRepBuilderAPI_MakeWire>;
        pub fn cadrs_wire_add(
            builder: Pin<&mut BRepBuilderAPI_MakeWire>,
            edge: &TopoDS_Edge,
        ) -> Result<()>;
        pub fn cadrs_wire_build(
            builder: Pin<&mut BRepBuilderAPI_MakeWire>,
        ) -> Result<UniquePtr<TopoDS_Wire>>;
        pub fn cadrs_face_from_wire(outer: &TopoDS_Wire) -> Result<UniquePtr<TopoDS_Face>>;
        pub fn cadrs_face_add_hole(
            face: &TopoDS_Face,
            hole: &TopoDS_Wire,
        ) -> Result<UniquePtr<TopoDS_Face>>;
        pub fn cadrs_face_surface_type(face: &TopoDS_Face) -> Result<i32>;

        // Solids
        pub fn cadrs_prism(
            shape: &TopoDS_Shape,
            dx: f64,
            dy: f64,
            dz: f64,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        #[allow(clippy::too_many_arguments)]
        pub fn cadrs_revol(
            shape: &TopoDS_Shape,
            ox: f64,
            oy: f64,
            oz: f64,
            dx: f64,
            dy: f64,
            dz: f64,
            angle: f64,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_boolean(
            a: &TopoDS_Shape,
            b: &TopoDS_Shape,
            op: i32,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_unify(shape: &TopoDS_Shape) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_fillet(
            shape: &TopoDS_Shape,
            edges: &TopTools_ListOfShape,
            radius: f64,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_chamfer(
            shape: &TopoDS_Shape,
            edges: &TopTools_ListOfShape,
            faces: &TopTools_ListOfShape,
            mode: i32,
            d1: f64,
            d2: f64,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_thick_solid(
            shape: &TopoDS_Shape,
            faces: &TopTools_ListOfShape,
            offset: f64,
            tolerance: f64,
        ) -> Result<UniquePtr<TopoDS_Shape>>;

        // Meshing
        pub fn cadrs_mesh(shape: &TopoDS_Shape, deflection: f64, angular: f64) -> Result<()>;
        pub fn cadrs_face_triangulation(
            face: &TopoDS_Face,
            positions: Pin<&mut CxxVector<f64>>,
            normals: Pin<&mut CxxVector<f64>>,
            indices: Pin<&mut CxxVector<i32>>,
        ) -> Result<()>;

        // Identity
        pub fn cadrs_shape_hash(shape: &TopoDS_Shape) -> u64;
        pub fn cadrs_shape_is_same(a: &TopoDS_Shape, b: &TopoDS_Shape) -> bool;

        pub fn cadrs_new_f64_vec() -> UniquePtr<CxxVector<f64>>;
        pub fn cadrs_new_i32_vec() -> UniquePtr<CxxVector<i32>>;

        // Modeling history (see `cadrs_history` in the header for the layout of `hist`).
        pub fn cadrs_prism_h(
            shape: &TopoDS_Shape,
            dx: f64,
            dy: f64,
            dz: f64,
            hist: Pin<&mut CxxVector<i32>>,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        #[allow(clippy::too_many_arguments)]
        pub fn cadrs_revol_h(
            shape: &TopoDS_Shape,
            ox: f64,
            oy: f64,
            oz: f64,
            dx: f64,
            dy: f64,
            dz: f64,
            angle: f64,
            hist: Pin<&mut CxxVector<i32>>,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_boolean_h(
            a: &TopoDS_Shape,
            b: &TopoDS_Shape,
            op: i32,
            hist: Pin<&mut CxxVector<i32>>,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_fillet_h(
            shape: &TopoDS_Shape,
            edges: &TopTools_ListOfShape,
            radius: f64,
            hist: Pin<&mut CxxVector<i32>>,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        #[allow(clippy::too_many_arguments)]
        pub fn cadrs_chamfer_h(
            shape: &TopoDS_Shape,
            edges: &TopTools_ListOfShape,
            faces: &TopTools_ListOfShape,
            mode: i32,
            d1: f64,
            d2: f64,
            hist: Pin<&mut CxxVector<i32>>,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_thick_solid_h(
            shape: &TopoDS_Shape,
            faces: &TopTools_ListOfShape,
            offset: f64,
            tolerance: f64,
            hist: Pin<&mut CxxVector<i32>>,
        ) -> Result<UniquePtr<TopoDS_Shape>>;

        // Topology queries (MapShapes order)
        pub fn cadrs_counts(shape: &TopoDS_Shape, out: Pin<&mut CxxVector<i32>>) -> Result<()>;
        pub fn cadrs_edges_info(shape: &TopoDS_Shape, out: Pin<&mut CxxVector<f64>>) -> Result<()>;
        pub fn cadrs_edge_faces(shape: &TopoDS_Shape, out: Pin<&mut CxxVector<i32>>) -> Result<()>;
        pub fn cadrs_vertices(
            shape: &TopoDS_Shape,
            points: Pin<&mut CxxVector<f64>>,
            edges: Pin<&mut CxxVector<i32>>,
        ) -> Result<()>;

        // Sub-shapes, compounds, thickening, rays and bounding boxes (cadrs P3.3)
        pub fn cadrs_sub_count(shape: &TopoDS_Shape, kind: i32) -> Result<i32>;
        pub fn cadrs_sub_shape(shape: &TopoDS_Shape, kind: i32, index: i32) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_compound(shapes: &TopTools_ListOfShape) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_thicken_h(
            shape: &TopoDS_Shape,
            offset: f64,
            hist: Pin<&mut CxxVector<i32>>,
        ) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_unify_h(shape: &TopoDS_Shape, hist: Pin<&mut CxxVector<i32>>) -> Result<UniquePtr<TopoDS_Shape>>;
        #[allow(clippy::too_many_arguments)]
        pub fn cadrs_ray_hits(
            shape: &TopoDS_Shape,
            ox: f64,
            oy: f64,
            oz: f64,
            dx: f64,
            dy: f64,
            dz: f64,
            out: Pin<&mut CxxVector<f64>>,
        ) -> Result<()>;
        pub fn cadrs_bbox(shape: &TopoDS_Shape, out: Pin<&mut CxxVector<f64>>) -> Result<()>;
    }
}
