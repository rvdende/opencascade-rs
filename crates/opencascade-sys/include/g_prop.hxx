#include <GProp_GProps.hxx>
#include <bindings_common.hxx>

inline std::unique_ptr<gp_Pnt> GProp_GProps_CentreOfMass(const GProp_GProps &props) {
  return std::unique_ptr<gp_Pnt>(new gp_Pnt(props.CentreOfMass()));
}

// The matrix of inertia about the centre of mass, axes parallel to the global ones (row-major:
// Ixx, Ixy, Ixz, Iyx, ...; OCCT's products of inertia already carry the tensor's minus sign).
inline void GProp_GProps_MatrixOfInertia(const GProp_GProps &props, rust::Slice<double> out) {
  gp_Mat m = props.MatrixOfInertia();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if ((size_t)(r * 3 + c) < out.size()) {
        out[r * 3 + c] = m.Value(r + 1, c + 1);
      }
    }
  }
}
