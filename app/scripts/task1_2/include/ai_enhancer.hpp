#pragma once
// ─────────────────────────────────────────────────────────────────────────
// ai_enhancer.hpp — step 8: send the fused Model3D plus the original
// images to an LLM and merge its refinement deltas back in.
//
// Two transports:
//   • Remote provider (OpenAI / Anthropic / Google) — invokes the existing
//     python/ai_caller.py which forwards to 127.0.0.1:5002/ai_call.
//     Provider receives the compact custom JSON model + a few thumbnails.
//   • Apple Intelligence (on-device FoundationModels) — invokes the Obj-C
//     bridge in objc/apple_intelligence.mm. No network. Text-only refinement
//     because on-device models cannot consume images via FoundationModels
//     today; they reason about the JSON only.
//
// Refinement schema (from the model's perspective, stable v1):
//   { "section_deltas":[{"id":int,"size_delta":[dl,dw,dh],"yaw_delta_deg":d}],
//     "plate_overrides":[{"id":int,"section_id":int,"face":"+x|...","u":f,"v":f}],
//     "warnings":["..."],
//     "confidence":0..1 }
//
// All values are advisory; we clamp the deltas to ±25% of the input
// dimension and refuse plate moves that would put a plate off-section.
// ─────────────────────────────────────────────────────────────────────────

#include "model3d.hpp"
#include <string>
#include <vector>

namespace mate {

struct AiEnhanceConfig {
    std::string provider;          // "openai" | "anthropic" | "google" | "apple"
    std::string model;             // provider-specific id
    std::string python_executable; // absolute path to python3 in app bundle
    std::string ai_caller_script;  // absolute path to ai_caller.py
    bool   on_device = false;      // use Apple Intelligence (FoundationModels)
    std::vector<std::string> thumbnail_paths;  // small JPEG/PNGs for context
    int max_clamp_pct = 25;        // ±% of dim that a refinement may change
};

struct AiEnhanceResult {
    bool ok = false;
    std::string error;
    std::string raw_response;
    std::string warning;           // non-fatal note from the model
    double confidence = 0.0;
    int section_deltas_applied = 0;
    int plate_overrides_applied = 0;
};

AiEnhanceResult enhanceWithAi(Model3D& m, const AiEnhanceConfig& cfg);

}  // namespace mate
