// pipe_sampson.cpp — Sampson-error LM endpoint refinement.
//
// Implementation notes:
//   • DLT init via cv::triangulatePoints (one point at a time is fine here;
//     we'd vectorise if this were a hot loop).
//   • Sampson cost (Hartley & Zisserman §12.6, eq. 12.6 generalised to two
//     views via reprojection):
//        C(X) = ||x_L - π(P1·X)||² + ||x_R - π(P2·X)||²
//     We minimise via dense Levenberg-Marquardt with central-difference
//     Jacobian — only 3 parameters per point so this is extremely fast.
//
#include "pipe_sampson.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace mate {

namespace {

cv::Point2d project(const cv::Mat& P, const cv::Point3d& X) {
    cv::Mat Xh = (cv::Mat_<double>(4,1) << X.x, X.y, X.z, 1.0);
    cv::Mat x  = P * Xh;
    double w = x.at<double>(2);
    // Inside the LM inner loop a tiny clamp is acceptable — the resulting
    // residual will be huge and the LM step rejected, so we never *use*
    // the bogus coordinate. The DLT init path above takes the strict
    // NaN-invalidation route because it is the seed of the optimisation.
    if (std::fabs(w) < 1e-12) w = (w < 0 ? -1e-12 : 1e-12);
    return { x.at<double>(0)/w, x.at<double>(1)/w };
}

bool finite3(const cv::Point3d& X) {
    return std::isfinite(X.x) && std::isfinite(X.y) && std::isfinite(X.z);
}

double residuals(const cv::Point3d& X,
                 const cv::Point2d& xL, const cv::Point2d& xR,
                 const cv::Mat& P1, const cv::Mat& P2) {
    cv::Point2d pL = project(P1, X), pR = project(P2, X);
    double rLx = pL.x - xL.x, rLy = pL.y - xL.y;
    double rRx = pR.x - xR.x, rRy = pR.y - xR.y;
    return rLx*rLx + rLy*rLy + rRx*rRx + rRy*rRy;
}

}  // namespace

cv::Point3d sampsonTriangulate(const cv::Point2d& xL, const cv::Point2d& xR,
                                const cv::Mat& P1, const cv::Mat& P2,
                                SampsonRefineReport* report) {
    // DLT init.
    cv::Mat ptsL(2, 1, CV_64F), ptsR(2, 1, CV_64F);
    ptsL.at<double>(0) = xL.x; ptsL.at<double>(1) = xL.y;
    ptsR.at<double>(0) = xR.x; ptsR.at<double>(1) = xR.y;
    // Triangulation requires CV_32F per OpenCV docs.
    cv::Mat ptsLf, ptsRf, P1f, P2f;
    ptsL.convertTo(ptsLf, CV_32F); ptsR.convertTo(ptsRf, CV_32F);
    P1.convertTo(P1f, CV_32F);    P2.convertTo(P2f, CV_32F);
    cv::Mat X4;
    cv::triangulatePoints(P1f, P2f, ptsLf, ptsRf, X4);
    double w = X4.at<float>(3);
    // True degeneracy: point at infinity along the baseline. Returning a
    // huge finite coordinate (the old behaviour) silently poisons every
    // downstream stage — the LM Jacobian explodes, RANSAC inlier counts
    // become meaningless, and the bundle prior weights blow up. Surface
    // the failure as NaN so the caller's std::isfinite() check trips.
    if (std::fabs(w) < 1e-12) {
        cv::Point3d X(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());
        if (report) { report->chi2_initial = report->chi2_final = -1; }
        return X;
    }
    cv::Point3d X(X4.at<float>(0)/w, X4.at<float>(1)/w, X4.at<float>(2)/w);
    if (!finite3(X) || X.z <= 0) {
        // Point behind camera or non-finite — invalidate.
        cv::Point3d Xn(std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::quiet_NaN());
        if (report) { report->chi2_initial = report->chi2_final = -1; }
        return Xn;
    }

    if (report) report->chi2_initial = residuals(X, xL, xR, P1, P2);

    // LM with 3 unknowns.
    double lambda = 1e-3;
    double prev   = residuals(X, xL, xR, P1, P2);
    int iters = 0;
    const double eps = 1e-7;
    for (int it = 0; it < 30; ++it) {
        iters = it + 1;
        // Numerical Jacobian (central differences, h scaled by point depth).
        double h = std::max(1e-6, std::fabs(X.z) * 1e-6);
        cv::Mat J(4, 3, CV_64F);
        cv::Mat r(4, 1, CV_64F);
        cv::Point2d pL = project(P1, X), pR = project(P2, X);
        r.at<double>(0) = pL.x - xL.x; r.at<double>(1) = pL.y - xL.y;
        r.at<double>(2) = pR.x - xR.x; r.at<double>(3) = pR.y - xR.y;
        for (int k = 0; k < 3; ++k) {
            cv::Point3d Xp = X, Xm = X;
            (k==0 ? Xp.x : k==1 ? Xp.y : Xp.z) += h;
            (k==0 ? Xm.x : k==1 ? Xm.y : Xm.z) -= h;
            cv::Point2d pLp=project(P1,Xp), pRp=project(P2,Xp);
            cv::Point2d pLm=project(P1,Xm), pRm=project(P2,Xm);
            J.at<double>(0,k) = (pLp.x - pLm.x) / (2*h);
            J.at<double>(1,k) = (pLp.y - pLm.y) / (2*h);
            J.at<double>(2,k) = (pRp.x - pRm.x) / (2*h);
            J.at<double>(3,k) = (pRp.y - pRm.y) / (2*h);
        }
        cv::Mat JtJ = J.t() * J;
        cv::Mat Jtr = J.t() * r;
        cv::Mat A   = JtJ + lambda * cv::Mat::diag(JtJ.diag());
        cv::Mat dx;
        if (!cv::solve(A, -Jtr, dx, cv::DECOMP_LU)) break;
        cv::Point3d Xn(X.x + dx.at<double>(0),
                        X.y + dx.at<double>(1),
                        X.z + dx.at<double>(2));
        double cur = residuals(Xn, xL, xR, P1, P2);
        if (cur < prev) {
            X = Xn;
            if (std::fabs(prev - cur) / std::max(1e-12, prev) < eps) {
                prev = cur;
                if (report) report->converged = true;
                break;
            }
            prev    = cur;
            lambda *= 0.5;
        } else {
            lambda *= 4.0;
            if (lambda > 1e8) break;
        }
    }
    if (report) { report->chi2_final = prev; report->iters_used = iters; }
    return X;
}

std::vector<cv::Point3d>
sampsonTriangulateMany(const std::vector<cv::Point2d>& xL,
                       const std::vector<cv::Point2d>& xR,
                       const cv::Mat& P1, const cv::Mat& P2) {
    std::vector<cv::Point3d> out; out.reserve(xL.size());
    size_t n = std::min(xL.size(), xR.size());
    for (size_t i = 0; i < n; ++i)
        out.push_back(sampsonTriangulate(xL[i], xR[i], P1, P2));
    return out;
}

}  // namespace mate
