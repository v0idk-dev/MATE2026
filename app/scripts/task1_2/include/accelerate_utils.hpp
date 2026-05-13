#pragma once
// ─────────────────────────────────────────────────────────────────────────
// accelerate_utils.hpp — Apple Accelerate (vDSP, BLAS, LAPACK) helpers.
//
// Apple Silicon only — Accelerate is the system framework that exposes
// AMX (matrix coprocessor) and the unified-memory-friendly BLAS/LAPACK.
// We use it for:
//   • PCA of 3-D point clouds (LAPACK SVD via dgesvd_)
//   • 3×3 / 4×4 matrix builds (cblas_dgemm)
//   • Median of large vectors (vDSP_vsorti for partition select)
//   • Vector multiply / add / scale (vDSP_vmul, vDSP_vsadd, vDSP_vsmul)
//
// All helpers degrade gracefully to pure-C++ implementations on non-Apple
// builds (the macros below select); the binary is Apple-only per spec but
// keeping the fallbacks lets the unit tests run anywhere.
// ─────────────────────────────────────────────────────────────────────────

#include "model3d.hpp"
#include <array>
#include <vector>

namespace mate::accel {

// Mean of N 3-D points. Apple build → vDSP_meanv on each component.
Vec3 mean3(const std::vector<Vec3>& pts);

// Median of N scalars. Apple build → vDSP_vsorti partition-select.
double median(std::vector<double> v);

// Three principal axes of a 3-D point cloud, sorted by descending
// eigenvalue. Apple build → LAPACK dgesvd_ on the centered data matrix.
struct Axes3 { Vec3 a0, a1, a2; };
Axes3 pcaAxes(const std::vector<Vec3>& pts);

// Solve the 2-D rigid alignment (rotation about Z + translation) that
// best maps `src` onto `dst` in least squares. Both vectors must be the
// same length. Returns yaw_rad and (tx, ty) plus the residual RMS in m.
// Apple build → LAPACK dgesvd_ inside Kabsch; CPU fallback uses Eigen-free
// hand-rolled SVD on a 2×2 covariance matrix.
struct YawXY { double yaw_rad = 0; double tx = 0; double ty = 0; double rms = 0; };
YawXY rigidAlignXY(const std::vector<Vec3>& src, const std::vector<Vec3>& dst);

// Apply a yaw + (tx, ty) to a 3-D point in place (Z is unchanged).
void applyYawXY(Vec3& p, const YawXY& t);

}  // namespace mate::accel
