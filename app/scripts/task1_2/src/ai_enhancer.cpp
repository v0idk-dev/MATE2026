// ai_enhancer.cpp — step 8: AI refinement.
#include "ai_enhancer.hpp"
#include "apple_intelligence.hpp"
#include "../json.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace mate {

namespace {

using nlohmann::json;

constexpr const char* kSystemPrompt =
    "You are reviewing a photogrammetric reconstruction of a small underwater PVC "
    "structure called a 'coral garden'. The structure has 3 cuboid SECTIONS attached "
    "to a common base, plus exactly 8 colored square PLATES (10cm x 10cm) attached to "
    "the sections. Total length is 1.0-2.5 m, total width approximately 36 cm.\n\n"
    "You will receive the reconstructed model as JSON. Use only physical plausibility "
    "(tank dimensions, plate count, plate size) to suggest small corrections. Return "
    "STRICT JSON of the form: "
    "{\"section_deltas\":[{\"id\":int,\"size_delta\":[dl,dw,dh],\"yaw_delta_deg\":d}],"
    " \"plate_overrides\":[{\"id\":int,\"section_id\":int,\"face\":str,\"u\":f,\"v\":f}],"
    " \"warnings\":[str], \"confidence\":0..1}. "
    "Keep deltas small (<25%% of original dimension). Reply with JSON only, no prose.";

bool runShell(const std::string& cmd, std::string& out) {
    FILE* p = ::popen(cmd.c_str(), "r");
    if (!p) return false;
    char buf[4096];
    while (size_t n = std::fread(buf, 1, sizeof(buf), p)) out.append(buf, n);
    int rc = ::pclose(p);
    return rc == 0;
}

// Find {...} JSON blob even if model wraps with markdown fences or prose.
std::string extractJsonBlob(const std::string& s) {
    size_t a = s.find('{');
    if (a == std::string::npos) return "";
    int depth = 0; size_t end = std::string::npos;
    for (size_t i = a; i < s.size(); ++i) {
        if (s[i] == '{') ++depth;
        else if (s[i] == '}') { if (--depth == 0) { end = i; break; } }
    }
    return end == std::string::npos ? "" : s.substr(a, end - a + 1);
}

}  // namespace

AiEnhanceResult enhanceWithAi(Model3D& m, const AiEnhanceConfig& cfg) {
    AiEnhanceResult r;

    const std::string model_json = toCustomJson(m, true);

    std::string raw;
    if (cfg.on_device || cfg.provider == "apple") {
        if (!apple_ai::isAvailable()) {
            r.error = "Apple Intelligence is not available on this device";
            return r;
        }
        std::string prompt = std::string(kSystemPrompt) +
                             "\n\nMODEL:\n" + model_json +
                             "\n\nREPLY WITH JSON ONLY:";
        std::string out;
        if (!apple_ai::generate(prompt, out)) {
            r.error = "Apple Intelligence call failed: " + out;
            return r;
        }
        raw = out;
    } else {
        // Spool inputs to temp files for ai_caller.py.
        std::string tmp_in   = std::tmpnam(nullptr); tmp_in  += ".json";
        std::string tmp_out  = std::tmpnam(nullptr); tmp_out += ".json";
        {
            std::ofstream f(tmp_in);
            json req = {
                {"system", kSystemPrompt},
                {"model_json", model_json},
                {"thumbnails", cfg.thumbnail_paths},
                {"provider", cfg.provider},
                {"model", cfg.model}};
            f << req.dump();
        }
        std::ostringstream cmd;
        cmd << '"' << cfg.python_executable << "\" \"" << cfg.ai_caller_script << "\""
            << " --input \"" << tmp_in << "\" --output \"" << tmp_out << "\"";
        std::string stderr_buf;
        if (!runShell(cmd.str() + " 2>&1", stderr_buf)) {
            r.error = "ai_caller failed: " + stderr_buf;
            return r;
        }
        std::ifstream f(tmp_out);
        std::stringstream ss; ss << f.rdbuf();
        raw = ss.str();
    }
    r.raw_response = raw;
    std::string blob = extractJsonBlob(raw);
    if (blob.empty()) { r.error = "model returned no JSON"; return r; }

    try {
        json j = json::parse(blob);
        r.confidence = j.value("confidence", 0.0);
        if (j.contains("warnings") && j["warnings"].is_array() && !j["warnings"].empty())
            r.warning = j["warnings"][0].get<std::string>();

        if (j.contains("section_deltas")) {
            for (const auto& d : j["section_deltas"]) {
                int id = d.value("id", -1);
                if (id < 0 || id >= (int)m.sections.size()) continue;
                auto& s = m.sections[id];
                if (d.contains("size_delta")) {
                    auto sd = d["size_delta"];
                    auto clamp = [&](double v, double base) {
                        const double maxd = std::abs(base) * (cfg.max_clamp_pct / 100.0);
                        return std::max(-maxd, std::min(maxd, v));
                    };
                    s.size.x = std::max(0.01, s.size.x + clamp(sd[0].get<double>(), s.size.x));
                    s.size.y = std::max(0.01, s.size.y + clamp(sd[1].get<double>(), s.size.y));
                    s.size.z = std::max(0.01, s.size.z + clamp(sd[2].get<double>(), s.size.z));
                }
                if (d.contains("yaw_delta_deg"))
                    s.yaw_deg += std::max(-15.0, std::min(15.0, d["yaw_delta_deg"].get<double>()));
                ++r.section_deltas_applied;
            }
        }
        if (j.contains("plate_overrides")) {
            for (const auto& d : j["plate_overrides"]) {
                int id = d.value("id", -1);
                if (id < 0 || id >= (int)m.plates.size()) continue;
                auto& p = m.plates[id];
                if (d.contains("section_id")) {
                    int sid = d["section_id"].get<int>();
                    if (sid >= 0 && sid < (int)m.sections.size()) p.section_id = sid;
                }
                if (d.contains("face")) p.face = d["face"].get<std::string>();
                if (d.contains("u")) p.u = std::clamp(d["u"].get<double>(), 0.0, 1.0);
                if (d.contains("v")) p.v = std::clamp(d["v"].get<double>(), 0.0, 1.0);
                p.corners_resolved = false;
                ++r.plate_overrides_applied;
            }
        }
        for (auto& p : m.plates) p.corners_resolved = false;
        resolvePlateCorners(m);
        recomputeBounds(m);
        r.ok = true;
        return r;
    } catch (const std::exception& e) {
        r.error = std::string("AI JSON parse failed: ") + e.what();
        return r;
    }
}

}  // namespace mate
