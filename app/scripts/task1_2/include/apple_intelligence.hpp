#pragma once
// ─────────────────────────────────────────────────────────────────────────
// apple_intelligence.hpp — Apple FoundationModels (on-device LLM) bridge.
//
// macOS 26+ on Apple Silicon. The system model runs entirely on the
// Neural Engine; no network, no API key. We expose a single sync call:
//
//   isAvailable() — true if the device has Apple Intelligence enabled
//                   and the FoundationModels framework is present.
//   generate(prompt, out_text) — blocking call into the system LLM.
//                   Returns true on success.
//
// For text-only refinement (the AI-enhance step in our pipeline) this
// is enough. Multimodal (image-input) on-device models are not yet
// exposed via FoundationModels as of this writing; if/when they are,
// a `generateWithImages()` overload will be added.
// ─────────────────────────────────────────────────────────────────────────

#include <string>

namespace mate::apple_ai {

bool isAvailable();

// Returns false if not available or the call fails. `out_text` receives
// the model's response on success, or an error description on failure.
bool generate(const std::string& prompt, std::string& out_text);

}  // namespace mate::apple_ai
