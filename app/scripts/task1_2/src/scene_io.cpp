#include "scene_io.hpp"
#include "json.hpp"

namespace mate {

using nlohmann::json;

namespace {

json point2fToJson(const cv::Point2f& p) { return json{ {"x", p.x}, {"y", p.y} }; }
json point3fToJson(const cv::Point3f& p) {
    return json{ {"x", p.x}, {"y", p.y}, {"z", p.z} };
}

}  // namespace

std::string sceneToJson(const SceneOutput& s, bool pretty) {
    json j;
    j["run"] = {
        {"mode", s.run.mode},
        {"version", s.run.version},
        {"timing_ms", s.run.timing_ms},
        {"warning", s.run.warning},
        {"error", s.run.error},
    };
    j["calibration"] = {
        {"present", s.calibration.present},
        {"rms_px", s.calibration.rms_px},
        {"avg_epipolar_err_px", s.calibration.avg_epipolar_err_px},
        {"baseline", s.calibration.baseline},
        {"baseline_unit", s.calibration.baseline_unit},
        {"image_width", s.calibration.image_width},
        {"image_height", s.calibration.image_height},
        {"pairs_used", s.calibration.pairs_used},
    };
    j["unit"] = s.unit;

    json plates_j = json::array();
    for (const auto& p : s.plates) {
        json pj;
        pj["id"] = p.id;
        pj["confidence"] = p.confidence;
        pj["center_left"]  = point2fToJson(p.center_left);
        pj["center_right"] = point2fToJson(p.center_right);
        json cornersL = json::array(), cornersR = json::array();
        for (int i = 0; i < 4; ++i) cornersL.push_back(point2fToJson(p.corners_left[i]));
        for (int i = 0; i < 4; ++i) cornersR.push_back(point2fToJson(p.corners_right[i]));
        pj["corners_left"]  = cornersL;
        pj["corners_right"] = cornersR;
        pj["has_3d"] = p.has_3d;
        if (p.has_3d) {
            pj["position_3d"] = point3fToJson(p.position_3d);
            json c3d = json::array();
            for (int i = 0; i < 4; ++i) c3d.push_back(point3fToJson(p.corners_3d[i]));
            pj["corners_3d"] = c3d;
        }
        plates_j.push_back(pj);
    }
    j["plates"] = plates_j;

    json pipes_j = json::array();
    for (const auto& p : s.pipes) {
        pipes_j.push_back({
            {"id", p.id},
            {"junction_a", p.junction_a},
            {"junction_b", p.junction_b},
            {"a", point3fToJson(p.a)},
            {"b", point3fToJson(p.b)},
            {"length", p.length},
        });
    }
    j["pipes"] = pipes_j;

    json junctions_j = json::array();
    for (const auto& jn : s.junctions) {
        junctions_j.push_back({
            {"id", jn.id},
            {"position", point3fToJson(jn.position)},
            {"degree", jn.degree},
        });
    }
    j["junctions"] = junctions_j;

    j["scale"] = {
        {"k", s.scale.k},
        {"confidence", s.scale.confidence},
        {"observations_used", s.scale.observations_used},
        {"source", s.scale.source},
        {"reason", s.scale.reason},
    };
    j["length"] = s.length;
    j["height"] = s.height;
    j["underwater"] = s.underwater;
    j["water_n"]    = s.water_n;

    // Dense MVS cloud (compact: parallel arrays for size).
    json dx = json::array(), dy = json::array(), dz = json::array();
    json dr = json::array(), dg = json::array(), db = json::array();
    for (size_t i = 0; i < s.dense_cloud_xyz.size(); ++i) {
        const auto& p = s.dense_cloud_xyz[i];
        dx.push_back(p.x); dy.push_back(p.y); dz.push_back(p.z);
        if (i < s.dense_cloud_rgb.size()) {
            const auto& c = s.dense_cloud_rgb[i];
            dr.push_back(c[2]); dg.push_back(c[1]); db.push_back(c[0]);
        }
    }
    j["dense_cloud"] = { {"x",dx},{"y",dy},{"z",dz},{"r",dr},{"g",dg},{"b",db} };

    return pretty ? j.dump(2) : j.dump();
}

}  // namespace mate
