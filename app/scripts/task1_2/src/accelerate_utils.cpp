// accelerate_utils.cpp — Apple Accelerate helpers, with portable fallbacks.
//
// On Apple builds (defined(__APPLE__)) every function delegates to vDSP /
// LAPACK so we get AMX-accelerated linear algebra. On non-Apple builds
// the same functions run as small hand-rolled C++ — the binary still
// compiles and the tests still pass.
#include "accelerate_utils.hpp"
#include <algorithm>
#include <cmath>

#if defined(__APPLE__)
  #include <Accelerate/Accelerate.h>
#endif

namespace mate::accel {

Vec3 mean3(const std::vector<Vec3>& pts) {
    if (pts.empty()) return {};
#if defined(__APPLE__)
    std::vector<double> X(pts.size()), Y(pts.size()), Z(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) { X[i]=pts[i].x; Y[i]=pts[i].y; Z[i]=pts[i].z; }
    double mx=0, my=0, mz=0;
    vDSP_meanvD(X.data(), 1, &mx, X.size());
    vDSP_meanvD(Y.data(), 1, &my, Y.size());
    vDSP_meanvD(Z.data(), 1, &mz, Z.size());
    return {mx, my, mz};
#else
    double sx=0, sy=0, sz=0;
    for (auto& p : pts) { sx+=p.x; sy+=p.y; sz+=p.z; }
    const double n = (double)pts.size();
    return { sx/n, sy/n, sz/n };
#endif
}

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    double m = v[mid];
    if ((v.size() & 1) == 0) {
        auto lo = std::max_element(v.begin(), v.begin() + mid);
        m = 0.5 * (m + *lo);
    }
    return m;
}

Axes3 pcaAxes(const std::vector<Vec3>& pts) {
    Axes3 R{ {1,0,0}, {0,1,0}, {0,0,1} };
    if (pts.size() < 3) return R;
    Vec3 mu = mean3(pts);

#if defined(__APPLE__)
    // Build the 3x3 covariance via cblas_dgemm.
    const int N = (int)pts.size();
    std::vector<double> A((size_t)3 * N);
    for (int i = 0; i < N; ++i) {
        A[0*N + i] = pts[i].x - mu.x;
        A[1*N + i] = pts[i].y - mu.y;
        A[2*N + i] = pts[i].z - mu.z;
    }
    double C[9] = {0};
    cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans,
                3, 3, N, 1.0/std::max(1,N-1),
                A.data(), N, A.data(), N, 0.0, C, 3);

    // SVD on 3x3 covariance to get eigenvectors (left singular vectors).
    double U[9], S[3], VT[9], work[64]; int lwork=64, info=0, m=3, n=3, lda=3, ldu=3, ldvt=3;
    char jobu = 'A', jobvt = 'N';
    dgesvd_(&jobu, &jobvt, &m, &n, C, &lda, S, U, &ldu, VT, &ldvt, work, &lwork, &info);
    if (info == 0) {
        R.a0 = { U[0], U[3], U[6] };  // column 0
        R.a1 = { U[1], U[4], U[7] };  // column 1
        R.a2 = { U[2], U[5], U[8] };  // column 2
    }
    return R;
#else
    // Power-iteration fallback — good enough for unit tests.
    double C[3][3] = {};
    for (auto& p : pts) {
        const double x = p.x-mu.x, y = p.y-mu.y, z = p.z-mu.z;
        C[0][0]+=x*x; C[0][1]+=x*y; C[0][2]+=x*z;
        C[1][1]+=y*y; C[1][2]+=y*z;
        C[2][2]+=z*z;
    }
    C[1][0]=C[0][1]; C[2][0]=C[0][2]; C[2][1]=C[1][2];
    auto power = [&](Vec3 init) {
        Vec3 v = init;
        for (int k=0; k<32; ++k) {
            Vec3 w = { C[0][0]*v.x + C[0][1]*v.y + C[0][2]*v.z,
                       C[1][0]*v.x + C[1][1]*v.y + C[1][2]*v.z,
                       C[2][0]*v.x + C[2][1]*v.y + C[2][2]*v.z };
            double n = std::sqrt(w.x*w.x + w.y*w.y + w.z*w.z);
            if (n < 1e-12) break;
            v = { w.x/n, w.y/n, w.z/n };
        }
        return v;
    };
    Vec3 a0 = power({1,0,0});
    // Deflate.
    auto deflate = [&](const Vec3& v){
        double l = v.x*(C[0][0]*v.x+C[0][1]*v.y+C[0][2]*v.z) +
                   v.y*(C[1][0]*v.x+C[1][1]*v.y+C[1][2]*v.z) +
                   v.z*(C[2][0]*v.x+C[2][1]*v.y+C[2][2]*v.z);
        C[0][0]-=l*v.x*v.x; C[0][1]-=l*v.x*v.y; C[0][2]-=l*v.x*v.z;
        C[1][0]-=l*v.y*v.x; C[1][1]-=l*v.y*v.y; C[1][2]-=l*v.y*v.z;
        C[2][0]-=l*v.z*v.x; C[2][1]-=l*v.z*v.y; C[2][2]-=l*v.z*v.z;
    };
    deflate(a0);
    Vec3 a1 = power({0,1,0});
    deflate(a1);
    Vec3 a2 = { a0.y*a1.z - a0.z*a1.y, a0.z*a1.x - a0.x*a1.z, a0.x*a1.y - a0.y*a1.x };
    R = {a0, a1, a2};
    return R;
#endif
}

YawXY rigidAlignXY(const std::vector<Vec3>& src, const std::vector<Vec3>& dst) {
    YawXY out;
    const size_t N = std::min(src.size(), dst.size());
    if (N == 0) return out;
    Vec3 cs = mean3(std::vector<Vec3>(src.begin(), src.begin()+N));
    Vec3 cd = mean3(std::vector<Vec3>(dst.begin(), dst.begin()+N));
    // 2×2 covariance H = Σ (s - cs)(d - cd)^T  in XY only.
    double H00=0, H01=0, H10=0, H11=0;
    for (size_t i=0; i<N; ++i) {
        const double sx = src[i].x - cs.x, sy = src[i].y - cs.y;
        const double dx = dst[i].x - cd.x, dy = dst[i].y - cd.y;
        H00 += sx*dx; H01 += sx*dy;
        H10 += sy*dx; H11 += sy*dy;
    }
    // Optimal rotation angle for 2D Procrustes:
    //   θ = atan2(H01 - H10, H00 + H11)
    out.yaw_rad = std::atan2(H01 - H10, H00 + H11);
    const double c = std::cos(out.yaw_rad), s = std::sin(out.yaw_rad);
    out.tx = cd.x - (c*cs.x - s*cs.y);
    out.ty = cd.y - (s*cs.x + c*cs.y);
    // Residual.
    double sse = 0;
    for (size_t i=0; i<N; ++i) {
        const double sx = c*src[i].x - s*src[i].y + out.tx;
        const double sy = s*src[i].x + c*src[i].y + out.ty;
        sse += (sx - dst[i].x)*(sx - dst[i].x) + (sy - dst[i].y)*(sy - dst[i].y);
    }
    out.rms = std::sqrt(sse / N);
    return out;
}

void applyYawXY(Vec3& p, const YawXY& t) {
    const double c = std::cos(t.yaw_rad), s = std::sin(t.yaw_rad);
    const double x = c*p.x - s*p.y + t.tx;
    const double y = s*p.x + c*p.y + t.ty;
    p.x = x; p.y = y;
}

}  // namespace mate::accel
