// pipe_graph.cpp — junction graph + connectivity gate.
#include "pipe_graph.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/flann.hpp>
#include <cmath>

namespace mate {

namespace {

cv::Point2d project3(const cv::Mat& P, const cv::Point3d& X) {
    cv::Mat Xh = (cv::Mat_<double>(4,1) << X.x, X.y, X.z, 1.0);
    cv::Mat x  = P * Xh;
    double w   = x.at<double>(2);
    return { x.at<double>(0)/w, x.at<double>(1)/w };
}

}  // namespace

PipeGraphResult buildPipeGraph(const std::vector<Cylinder3D>& cyls,
                                const cv::Mat& P1,
                                const PipeGraphConfig& cfg) {
    PipeGraphResult R;
    if (cyls.empty()) return R;

    // 1. Collect endpoints + their pipe index + which-end (0|1).
    struct EP { cv::Point3d p; int pipe; int end; };
    std::vector<EP> eps;
    eps.reserve(cyls.size() * 2);
    for (size_t i = 0; i < cyls.size(); ++i) {
        eps.push_back({cyls[i].endpoint_a, (int)i, 0});
        eps.push_back({cyls[i].endpoint_b, (int)i, 1});
    }

    // 2. KD-tree cluster.
    cv::Mat data((int)eps.size(), 3, CV_32F);
    for (size_t i = 0; i < eps.size(); ++i) {
        data.at<float>((int)i, 0) = (float)eps[i].p.x;
        data.at<float>((int)i, 1) = (float)eps[i].p.y;
        data.at<float>((int)i, 2) = (float)eps[i].p.z;
    }
    cv::flann::KDTreeIndexParams params(1);
    cv::flann::Index kdtree(data, params);

    std::vector<int> cluster_of(eps.size(), -1);
    std::vector<GraphJunction> junctions;
    for (size_t i = 0; i < eps.size(); ++i) {
        if (cluster_of[i] != -1) continue;
        int cid = (int)junctions.size();
        cluster_of[i] = cid;
        GraphJunction j;
        j.position = eps[i].p;

        std::vector<int> idx; std::vector<float> dist;
        cv::Mat q = (cv::Mat_<float>(1,3) << eps[i].p.x, eps[i].p.y, eps[i].p.z);
        // Cap at eps.size() instead of 16: in dense PVC junctions (5+ pipes
        // meeting at one tee/elbow stack) the 16-cap silently dropped real
        // endpoints and merged them into the wrong cluster.
        kdtree.radiusSearch(q, idx, dist, (float)(cfg.merge_radius_m*cfg.merge_radius_m), (int)eps.size());
        cv::Point3d sum(0,0,0); int cnt = 0;
        for (size_t k = 0; k < idx.size(); ++k) {
            int ii = idx[k];
            if (ii < 0 || ii >= (int)eps.size()) continue;
            if (dist[k] > cfg.merge_radius_m*cfg.merge_radius_m) continue;
            cluster_of[ii] = cid;
            sum.x += eps[ii].p.x; sum.y += eps[ii].p.y; sum.z += eps[ii].p.z; ++cnt;
        }
        if (cnt > 0) j.position = cv::Point3d(sum.x/cnt, sum.y/cnt, sum.z/cnt);
        junctions.push_back(j);
    }

    // 3. Build pipes and assign junction indices.
    std::vector<GraphPipe> pipes;
    for (size_t i = 0; i < cyls.size(); ++i) {
        GraphPipe gp;
        gp.cyl = cyls[i];
        gp.junction_a = cluster_of[2*i + 0];
        gp.junction_b = cluster_of[2*i + 1];
        pipes.push_back(gp);
        if (gp.junction_a >= 0) {
            junctions[gp.junction_a].pipe_indices.push_back((int)i);
            junctions[gp.junction_a].degree++;
        }
        if (gp.junction_b >= 0) {
            junctions[gp.junction_b].pipe_indices.push_back((int)i);
            junctions[gp.junction_b].degree++;
        }
    }

    // 4. Optional 2D consistency: project both ends; require either close to
    // *another* projected endpoint within merge_radius_px (degree-2 in 2D)
    // OR allow degree-1 if the pipe is on the boundary of the structure.
    // For simplicity: just keep the 3D-degree gate. (2D check can be added
    // by projecting all endpoints and running a 2D KD-tree.)
    (void)P1; (void)cfg.require_2d_check;

    // 5. Reject pipes whose BOTH endpoints are isolated (degree==1, i.e.
    // only this pipe touches the cluster). A pipe with at least one
    // shared junction passes — the structure is a connected graph.
    std::vector<GraphPipe> kept;
    for (auto& p : pipes) {
        bool a_shared = (p.junction_a >= 0 && junctions[p.junction_a].degree >= 2);
        bool b_shared = (p.junction_b >= 0 && junctions[p.junction_b].degree >= 2);
        if (a_shared || b_shared) kept.push_back(p);
        else R.rejected_isolated++;
    }
    R.pipes     = std::move(kept);
    R.junctions = std::move(junctions);
    return R;
}

}  // namespace mate
