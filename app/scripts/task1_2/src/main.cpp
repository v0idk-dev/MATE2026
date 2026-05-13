// main.cpp — single CLI entry point for the overhauled Task 1.2 pipeline.
//
// Usage (one logical line; backslashes shown for readability only):
//   task1_2
//     --pair leftA.jpg rightA.jpg
//     [--pair leftB.jpg rightB.jpg ...]
//     --left-calib left.yaml --right-calib right.yaml
//     [--stereo stereo.yaml]
//     [--out model.json] [--glb model.glb] [--obj model.obj]
//     [--debug-dir debug/]
//     [--target-hue 135] [--hue-tol 25] [--expected-plates 8]
//     [--underwater] [--water-n 1.333]
//     [--ai-enhance --ai-provider openai --ai-model gpt-4o-mini
//        --python /path/to/python --ai-script /path/to/ai_caller.py]
//     [--apple-intelligence]
//     [--manual-width-m 0.36]
//
// Writes Custom JSON to stdout if --out not given. Prints a one-line
// human-readable summary to stderr.
#include "pipeline.hpp"
#include "model3d.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace mate;

static void usage() {
    std::cerr << "usage: task1_2 --pair L R [--pair L R ...] --left-calib f --right-calib f "
                 "[--stereo f] [--out model.json] [--glb model.glb] [--obj model.obj] "
                 "[--debug-dir d] [--target-hue h] [--hue-tol t] [--expected-plates n] "
                 "[--underwater] [--water-n n] "
                 "[--ai-enhance --ai-provider p --ai-model m --python py --ai-script s] "
                 "[--apple-intelligence] [--manual-width-m w] "
                 "[--rig-baseline-m b] [--no-auto-calib] [--engine plate|pipe]\n";
}

// Stamped at compile time so we can tell at-a-glance whether a running
// binary actually contains the latest source changes. If the build date
// printed at startup is older than your last edit, you forgot to rebuild.
#ifndef TASK1_2_BUILD_STAMP
#define TASK1_2_BUILD_STAMP (__DATE__ " " __TIME__)
#endif

int main(int argc, char** argv) {
    std::cerr << "task1_2 binary build: " << TASK1_2_BUILD_STAMP
              << "  (rev: matching-fallback-T1..T4 + manhattan-autocalib)\n";
    PipelineInput in;
    std::string out_json, out_glb, out_obj;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](int n){ if (i + n >= argc) { usage(); std::exit(2); } };
        if      (a == "--pair")              { need(2); in.pairs.push_back({argv[i+1], argv[i+2], "", "", ""}); i += 2; }
        else if (a == "--left-calib")        { need(1); in.left_calib_yaml = argv[++i]; }
        else if (a == "--right-calib")       { need(1); in.right_calib_yaml = argv[++i]; }
        else if (a == "--stereo")            { need(1); in.stereo_extrinsics_yaml = argv[++i]; }
        else if (a == "--out")               { need(1); out_json = argv[++i]; }
        else if (a == "--glb")               { need(1); out_glb  = argv[++i]; }
        else if (a == "--obj")               { need(1); out_obj  = argv[++i]; }
        else if (a == "--debug-dir")         { need(1); in.debug_dir = argv[++i]; }
        else if (a == "--target-hue")        { need(1); in.plate_target_h = std::stoi(argv[++i]); }
        else if (a == "--hue-tol")           { need(1); in.plate_hue_tolerance = std::stoi(argv[++i]); }
        else if (a == "--expected-plates")   { need(1); in.expected_plates = std::stoi(argv[++i]); }
        else if (a == "--plate-side-m")      { need(1); in.plate_side_m   = std::stod(argv[++i]); }
        else if (a == "--underwater")        { in.underwater = true; }
        else if (a == "--water-n")           { need(1); in.water_n = std::stod(argv[++i]); }
        else if (a == "--ai-enhance")        { in.ai_enhance = true; }
        else if (a == "--ai-provider")       { need(1); in.ai_provider = argv[++i]; }
        else if (a == "--ai-model")          { need(1); in.ai_model = argv[++i]; }
        else if (a == "--python")            { need(1); in.ai_caller_executable = argv[++i]; }
        else if (a == "--ai-script")         { need(1); in.ai_caller_script = argv[++i]; }
        else if (a == "--apple-intelligence"){ in.use_apple_intelligence = true; }
        else if (a == "--manual-width-m")    { need(1); in.manual_total_width_m = std::stod(argv[++i]); }
        else if (a == "--rig-baseline-m")    { need(1); in.rig_baseline_m       = std::stod(argv[++i]); }
        else if (a == "--no-auto-calib")     { in.rig_baseline_m = 0.0; }
        else if (a == "--no-metal")          { in.use_metal = false; }
        else if (a == "--no-accelerate")     { in.use_accelerate = false; }
        else if (a == "--no-vision")         { in.use_vision = false; }
        else if (a == "--engine")            { need(1); std::string e = argv[++i]; if (e == "pipe") in.engine_pipe = true; }
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else                                 { std::cerr << "unknown arg: " << a << "\n"; usage(); return 2; }
    }
    if (in.pairs.empty()) { usage(); return 2; }

    PipelineOutput r = runPipeline(in,
        [](int step, const std::string& label, double frac) {
            std::cerr << "[" << step << "/" << 9 << "] " << label
                      << " (" << (int)(frac*100) << "%)\n";
        });

    if (!r.error.empty()) {
        std::cerr << "ERROR: " << r.error << "\n";
        return 1;
    }
    std::string js = toCustomJson(r.model, true);
    if (out_json.empty()) std::cout << js << "\n";
    else { std::ofstream f(out_json); f << js; }

    // Per-pair detection sidecar — lets the web UI tell the user *why*
    // the fused model is empty (wrong plate hue → 0 plates everywhere,
    // bad lighting → 0 pipes, rectify failed → error string set, etc.).
    // Hand-rolled JSON to avoid pulling nlohmann into main.cpp.
    if (!out_json.empty()) {
        auto esc = [](const std::string& s) {
            std::string o; o.reserve(s.size() + 2);
            for (char c : s) {
                if (c == '"' || c == '\\') { o.push_back('\\'); o.push_back(c); }
                else if (c == '\n') o += "\\n";
                else if (c == '\r') o += "\\r";
                else if ((unsigned char)c < 0x20) { /* drop */ }
                else o.push_back(c);
            }
            return o;
        };
        std::string pp_path = out_json;
        auto slash = pp_path.find_last_of("/\\");
        std::string dir = (slash == std::string::npos) ? "" : pp_path.substr(0, slash + 1);
        std::ofstream pf(dir + "per_pair.json");
        pf << "[";
        for (size_t i = 0; i < r.per_pair.size(); ++i) {
            const auto& d = r.per_pair[i];
            if (i) pf << ",";
            pf << "{\"pair_index\":"     << d.pair_index
               << ",\"n_plates_left\":"  << d.n_plates_left
               << ",\"n_plates_right\":" << d.n_plates_right
               << ",\"n_pipes_left\":"   << d.n_pipes_left
               << ",\"n_pipes_right\":"  << d.n_pipes_right
               << ",\"error\":\""        << esc(d.error) << "\"}";
        }
        pf << "]";
    }

    if (!out_glb.empty()) {
        std::string glb = toGlbBinary(r.model);
        std::ofstream f(out_glb, std::ios::binary); f.write(glb.data(), glb.size());
    }
    if (!out_obj.empty()) {
        auto [obj, mtl] = toObjMtl(r.model);
        { std::ofstream f(out_obj); f << obj; }
        std::string mtl_path = out_obj.substr(0, out_obj.find_last_of('.')) + ".mtl";
        { std::ofstream f(mtl_path); f << mtl; }
    }

    std::cerr << "ok pairs=" << r.model.n_pairs_used
              << " sections=" << r.model.sections.size()
              << " plates=" << r.model.plates.size()
              << " L=" << r.model.total_length << "m"
              << " W=" << r.model.total_width  << "m"
              << " H=" << r.model.total_height << "m"
              << " (" << r.timing_ms_total << " ms)\n";
    return 0;
}
