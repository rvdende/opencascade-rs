//! STEP and IGES with assembly structure and names, through OpenCASCADE's XDE (cadrs branch).
//!
//! [`read_step`] / [`read_iges`] give a file's **parts** (each distinct shape once, with its
//! product name) and their **occurrences** (which part, where, and the instance's name), in
//! millimetres. A file without assemblies has one occurrence per part, at the identity.
//! [`Writer`] writes parts with names and, when instances are added, one assembly holding them.

use opencascade_sys as ffi;

use crate::{primitives::Shape, Error};

fn occt(e: cxx::Exception) -> Error {
    Error::Occt(e.what().to_string())
}

/// A part of an imported file.
pub struct XdePart {
    pub shape: Shape,
    pub name: String,
}

/// One use of a part: the part's index, its placement as a 3×4 matrix `[R | t]` (rows first)
/// in the file's coordinates, and the instance's name.
#[derive(Debug, Clone, PartialEq)]
pub struct XdeOccurrence {
    pub part: usize,
    pub matrix: [f64; 12],
    pub name: String,
}

/// An imported file.
pub struct XdeModel {
    /// The first top-level product's name.
    pub name: String,
    pub parts: Vec<XdePart>,
    pub occurrences: Vec<XdeOccurrence>,
}

fn read(path: &std::path::Path, iges: bool) -> Result<XdeModel, Error> {
    let x = ffi::cadrs_xde::cadrs_read_xde(&path.to_string_lossy(), iges).map_err(occt)?;
    let mut parts = Vec::new();
    for i in 0..ffi::cadrs_xde::cadrs_xde_part_count(&x) {
        let inner = ffi::cadrs_xde::cadrs_xde_part_shape(&x, i).map_err(occt)?;
        let name = ffi::cadrs_xde::cadrs_xde_part_name(&x, i).map_err(occt)?;
        parts.push(XdePart { shape: Shape { inner }, name });
    }
    let mut occurrences = Vec::new();
    for i in 0..ffi::cadrs_xde::cadrs_xde_occurrence_count(&x) {
        let part = ffi::cadrs_xde::cadrs_xde_occurrence_part(&x, i).map_err(occt)? as usize;
        let mut matrix = [0.0; 12];
        ffi::cadrs_xde::cadrs_xde_occurrence_matrix(&x, i, &mut matrix).map_err(occt)?;
        let name = ffi::cadrs_xde::cadrs_xde_occurrence_name(&x, i).map_err(occt)?;
        occurrences.push(XdeOccurrence { part, matrix, name });
    }
    Ok(XdeModel { name: ffi::cadrs_xde::cadrs_xde_name(&x), parts, occurrences })
}

/// Reads a STEP file with its assembly structure and names.
pub fn read_step(path: impl AsRef<std::path::Path>) -> Result<XdeModel, Error> {
    read(path.as_ref(), false)
}

/// Reads an IGES file (IGES has no assemblies: each shape is a part used once).
pub fn read_iges(path: impl AsRef<std::path::Path>) -> Result<XdeModel, Error> {
    read(path.as_ref(), true)
}

/// Writes parts, and optionally an assembly of their instances, to STEP or IGES.
pub struct Writer {
    inner: cxx::UniquePtr<ffi::cadrs_xde::CadrsXdeWriter>,
}

impl Writer {
    /// A writer whose assembly (if any instances are added) is called `name`.
    pub fn new(name: &str) -> Result<Self, Error> {
        Ok(Self { inner: ffi::cadrs_xde::cadrs_xde_writer_new(name).map_err(occt)? })
    }

    /// Adds a part; its index for [`Writer::add_instance`].
    pub fn add_part(&mut self, shape: &Shape, name: &str) -> Result<usize, Error> {
        ffi::cadrs_xde::cadrs_xde_writer_add_part(self.inner.pin_mut(), &shape.inner, name)
            .map(|i| i as usize)
            .map_err(occt)
    }

    /// Adds an instance of part `part` at `matrix` (`[R | t]`, rows first; R a rotation).
    pub fn add_instance(&mut self, part: usize, matrix: &[f64; 12], name: &str) -> Result<(), Error> {
        ffi::cadrs_xde::cadrs_xde_writer_add_instance(self.inner.pin_mut(), part as i32, matrix, name)
            .map_err(occt)
    }

    /// Writes STEP (AP214, mm) with product names.
    pub fn write_step(mut self, path: impl AsRef<std::path::Path>) -> Result<(), Error> {
        ffi::cadrs_xde::cadrs_xde_writer_write(self.inner.pin_mut(), &path.as_ref().to_string_lossy(), false)
            .map_err(occt)
    }

    /// Writes IGES (mm, solids as MSBO solids); instances are written where they are.
    pub fn write_iges(mut self, path: impl AsRef<std::path::Path>) -> Result<(), Error> {
        ffi::cadrs_xde::cadrs_xde_writer_write(self.inner.pin_mut(), &path.as_ref().to_string_lossy(), true)
            .map_err(occt)
    }
}

/// The shape healed by OpenCASCADE's `ShapeFix_Shape` (gaps, orientations, missing curves), as
/// imported shapes often need.
pub fn fix_shape(shape: &Shape) -> Result<Shape, Error> {
    let inner = ffi::cadrs_xde::cadrs_shape_fix(&shape.inner).map_err(occt)?;
    Ok(Shape { inner })
}
