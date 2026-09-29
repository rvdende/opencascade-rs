//! STEP and IGES through OpenCASCADE's XDE, with assembly structure and names (cadrs branch,
//! see `include/cadrs_xde.hxx`). Every fallible call returns a `Result`.
pub use inner::*;

#[cxx::bridge]
mod inner {
    unsafe extern "C++" {
        include!("opencascade-sys/include/cadrs_xde.hxx");

        type TopoDS_Shape = crate::topo_ds::TopoDS_Shape;

        /// A file read through XDE: its parts and their occurrences.
        type CadrsXde;
        pub fn cadrs_read_xde(path: &str, iges: bool) -> Result<UniquePtr<CadrsXde>>;
        pub fn cadrs_xde_name(x: &CadrsXde) -> String;
        pub fn cadrs_xde_part_count(x: &CadrsXde) -> i32;
        pub fn cadrs_xde_part_shape(x: &CadrsXde, i: i32) -> Result<UniquePtr<TopoDS_Shape>>;
        pub fn cadrs_xde_part_name(x: &CadrsXde, i: i32) -> Result<String>;
        pub fn cadrs_xde_occurrence_count(x: &CadrsXde) -> i32;
        pub fn cadrs_xde_occurrence_part(x: &CadrsXde, i: i32) -> Result<i32>;
        pub fn cadrs_xde_occurrence_name(x: &CadrsXde, i: i32) -> Result<String>;
        pub fn cadrs_xde_occurrence_matrix(x: &CadrsXde, i: i32, out: &mut [f64]) -> Result<()>;

        /// An XCAF document being written.
        type CadrsXdeWriter;
        pub fn cadrs_xde_writer_new(name: &str) -> Result<UniquePtr<CadrsXdeWriter>>;
        pub fn cadrs_xde_writer_add_part(
            w: Pin<&mut CadrsXdeWriter>,
            shape: &TopoDS_Shape,
            name: &str,
        ) -> Result<i32>;
        pub fn cadrs_xde_writer_add_instance(
            w: Pin<&mut CadrsXdeWriter>,
            part: i32,
            matrix: &[f64],
            name: &str,
        ) -> Result<()>;
        pub fn cadrs_xde_writer_write(w: Pin<&mut CadrsXdeWriter>, path: &str, iges: bool) -> Result<()>;

        /// Shape healing (ShapeFix_Shape) of an imported shape.
        pub fn cadrs_shape_fix(shape: &TopoDS_Shape) -> Result<UniquePtr<TopoDS_Shape>>;
    }
}
