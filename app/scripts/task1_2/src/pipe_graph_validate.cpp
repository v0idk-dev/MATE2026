// pipe_graph_validate.cpp — see header for description.
#include "pipe_graph_validate.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <sstream>

namespace mate {

namespace {

double snapRadius(double r, const std::vector<double>& cat, double* residual) {
    if (cat.empty()) { if (residual) *residual = 0; return r; }
    double best = cat[0]; double bestd = std::fabs(r - cat[0]);
    for (size_t i = 1; i < cat.size(); ++i) {
        double d = std::fabs(r - cat[i]);
        if (d < bestd) { bestd = d; best = cat[i]; }
    }
    if (residual) *residual = bestd;
    return best;
}

}  // namespace

PipeGraphValidateReport
validatePipeGraph(const PipeGraphResult& g, const PipeGraphValidateConfig& cfg) {
    PipeGraphValidateReport r;
    r.num_pipes     = (int)g.pipes.size();
    r.num_junctions = (int)g.junctions.size();
    r.snapped_radii_m.reserve(g.pipes.size());

    // (1) Length sanity + (2) radius catalog snap.
    for (size_t i = 0; i < g.pipes.size(); ++i) {
        const auto& cyl = g.pipes[i].cyl;
        double L = cyl.length_m;
        if (!std::isfinite(L) || L <= 0 || L > cfg.max_pipe_length_m) {
            r.drop_pipe_indices.push_back((int)i);
            r.num_long_pipes_flagged++;
            std::ostringstream os;
            os << "pipe " << i << " length " << L << " m exceeds cap "
               << cfg.max_pipe_length_m << " m";
            r.warnings.push_back(os.str());
        }
        double resid = 0;
        double snap = snapRadius(cyl.radius_m, cfg.radius_catalog_m, &resid);
        r.snapped_radii_m.push_back(snap);
        if (resid > cfg.radius_outlier_m) {
            r.num_radius_outliers++;
            std::ostringstream os;
            os << "pipe " << i << " radius " << cyl.radius_m * 1000.0
               << " mm is " << resid * 1000.0
               << " mm off catalog (snap → " << snap * 1000.0 << " mm)";
            r.warnings.push_back(os.str());
        }
    }

    // (4) Degree distribution.
    for (const auto& j : g.junctions) {
        if (j.degree > r.max_observed_degree) r.max_observed_degree = j.degree;
        if (j.degree > cfg.max_junction_degree) {
            std::ostringstream os;
            os << "junction degree " << j.degree
               << " exceeds plausible cap " << cfg.max_junction_degree
               << " (merge_radius_m may be too large)";
            r.warnings.push_back(os.str());
        }
    }

    // (3) Connectivity via BFS over the junction-pipe-junction graph.
    if (!g.junctions.empty()) {
        std::vector<std::vector<int>> adj(g.junctions.size());
        for (size_t pi = 0; pi < g.pipes.size(); ++pi) {
            int a = g.pipes[pi].junction_a, b = g.pipes[pi].junction_b;
            if (a >= 0 && b >= 0 &&
                a < (int)adj.size() && b < (int)adj.size()) {
                adj[a].push_back(b); adj[b].push_back(a);
            }
        }
        std::vector<char> seen(g.junctions.size(), 0);
        int comps = 0;
        for (size_t s = 0; s < g.junctions.size(); ++s) {
            if (seen[s]) continue;
            ++comps;
            std::queue<int> q; q.push((int)s); seen[s] = 1;
            while (!q.empty()) {
                int u = q.front(); q.pop();
                for (int v : adj[u]) if (!seen[v]) { seen[v] = 1; q.push(v); }
            }
        }
        r.num_connected_components = comps;
        if (comps > 1) {
            std::ostringstream os;
            os << "graph has " << comps
               << " disconnected components — at least one floating pipe";
            r.warnings.push_back(os.str());
        }
    }

    // De-duplicate drop indices.
    std::sort(r.drop_pipe_indices.begin(), r.drop_pipe_indices.end());
    r.drop_pipe_indices.erase(
        std::unique(r.drop_pipe_indices.begin(), r.drop_pipe_indices.end()),
        r.drop_pipe_indices.end());
    return r;
}

}  // namespace mate
