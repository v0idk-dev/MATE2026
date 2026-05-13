#pragma once
// ─────────────────────────────────────────────────────────────────────────
// model3d.hpp — the canonical 3-section coral-garden model.
//
// The PVC structure is rectilinear and made of cuboid "sections" plus a
// set of identically-sized colored plates attached to those sections.
// Per the sketch the rig has 3 sections of different heights sitting on a
// shared base; per the spec there are exactly 8 plates of identical color
// and 10 cm × 10 cm size.
//
// This is the format we use:
//   • internally between pipeline stages (replaces the per-segment Wireframe
//     in the old code — sections + plates are MUCH more compact, and they
//     match the structure the AI prompts also reason about);
//   • for AI enhancement (sent to the model as plain JSON, ~1–2 KB);
//   • for in-app persistence (Custom JSON export/import);
//   • for Blender-compatible export (GLB / glTF 2.0 + OBJ fallback —
//     written by the C++ side so the Electron app does not need to know
//     anything about 3D file formats).
//
// All coordinates are in METERS in the model's own local frame:
//   • +X: along the long axis of the structure ("length")
//   • +Y: across the structure ("width")
//   • +Z: up ("height")
// The pipeline transforms left-camera-frame triangulations into this frame
// before populating Model3D (see per_pair_model.cpp).
// ─────────────────────────────────────────────────────────────────────────

#include <array>
#include <string>
#include <vector>

namespace mate {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    // Index accessors for legacy code that uses Vec3 like a fixed-size
    // array. 0=x, 1=y, 2=z. Out-of-range returns/writes x as a fail-soft.
    double& operator[](int i)       { return i==0?x : i==1?y : z; }
    double  operator[](int i) const { return i==0?x : i==1?y : z; }
    // Iterators so range-for (auto& v : vec3) works for legacy code that
    // wants to scale every component (e.g. bundle_adjust.cpp).
    double* begin()             { return &x; }
    double* end()               { return &x + 3; }
    const double* begin() const { return &x; }
    const double* end()   const { return &x + 3; }
    // Container-like size, so generic code (and static_asserts) treat
    // Vec3 as a fixed-3-element sequence. constexpr so it can be used
    // in constant expressions.
    static constexpr std::size_t size() { return 3; }
};

// Plate face identifier — provided as both an enum (used by legacy code
// such as refine_scale.cpp) and a string (the canonical form stored in
// Plate::face for JSON portability). `parseFace` does the bridge.
enum class PlateFace { PosX, NegX, PosY, NegY, PosZ, NegZ, Unknown };

inline PlateFace parseFace(const std::string& s) {
    if (s == "+x") return PlateFace::PosX;
    if (s == "-x") return PlateFace::NegX;
    if (s == "+y") return PlateFace::PosY;
    if (s == "-y") return PlateFace::NegY;
    if (s == "+z") return PlateFace::PosZ;
    if (s == "-z") return PlateFace::NegZ;
    return PlateFace::Unknown;
}

inline const char* faceToString(PlateFace f) {
    switch (f) {
        case PlateFace::PosX: return "+x";
        case PlateFace::NegX: return "-x";
        case PlateFace::PosY: return "+y";
        case PlateFace::NegY: return "-y";
        case PlateFace::PosZ: return "+z";
        case PlateFace::NegZ: return "-z";
        default:              return "?";
    }
}

inline Vec3 vec3(double x, double y, double z) { return {x, y, z}; }

// One rectilinear cuboid section — the base of one part of the structure.
// `origin` is the center of the BOTTOM face. `size` is (length_x, width_y,
// height_z). `yaw_deg` lets a section rotate about Z relative to the parent
// model frame (typically 0 because we already align the principal axis).
struct Section {
    int id = -1;
    Vec3 origin{};         // m
    Vec3 size{};           // m  (l, w, h)
    double yaw_deg = 0.0;  // rotation about +Z, degrees
    double confidence = 0.0;
};

// One plate stuck to one face of one section. We encode the *attachment*
// (section id + face + uv on that face) so the position is implicitly tied
// to the section — moving a section moves its plates. We also cache the
// resolved 3-D corner positions for renderers that don't want to redo the
// math.
//
// Faces: "+x", "-x", "+y", "-y", "+z" (top), "-z" (bottom).
struct Plate {
    int id = -1;
    int section_id = -1;
    std::string face = "+z";
    double u = 0.5;        // 0..1 along the face's first tangent axis
    double v = 0.5;        // 0..1 along the face's second tangent axis
    double side_m = 0.10;  // physical edge length, default per spec
    double confidence = 0.0;
    // Cached corner positions in model frame (m), order TL, TR, BR, BL with
    // respect to the face's local tangents. -1/empty if not yet resolved.
    std::array<Vec3, 4> corners{};
    bool corners_resolved = false;
};

struct ScaleInfo {
    double k = 1.0;          // multiplicative scale already applied
    std::string source;      // "stereo" | "plate-prior" | "manual" | "ai"
    double confidence = 0.0; // 0..1
    std::string reason;
};

struct CalibrationInfo {
    bool present = false;
    double rms_px = -1.0;
    double avg_epipolar_err_px = -1.0;
    double baseline_m = 0.0;
    int    image_width  = 0;
    int    image_height = 0;
    int    pairs_used   = 0;
};

struct Model3D {
    int version = 1;
    std::string unit = "m";
    std::vector<Section> sections;
    std::vector<Plate>   plates;
    // Top-level bounding dimensions (m).
    double total_length = 0.0;
    double total_width  = 0.0;
    double total_height = 0.0;
    // Provenance.
    int    n_pairs_used = 0;
    ScaleInfo scale;
    CalibrationInfo calibration;
    std::string warning;
    long long timing_ms = 0;
    // Optional: raw triangulated 3D plate centers in model frame, kept for
    // debugging / re-fusion. Not required for rendering.
    std::vector<Vec3> raw_plate_centers;
};

// Resolve cached `corners` for every plate that doesn't have them yet.
// Idempotent.
void resolvePlateCorners(Model3D& m);

// Recompute total_length / total_width / total_height from sections.
void recomputeBounds(Model3D& m);

// Apply a uniform multiplicative scale `k` to every metric quantity (sizes,
// origins, plate corners, totals). Updates m.scale.
void applyScale(Model3D& m, double k, const std::string& source,
                const std::string& reason, double confidence);

// Custom JSON — the compact wire/persist format. ~1–2 KB for a typical
// 3-section / 8-plate structure. Stable schema with an integer `version`.
std::string toCustomJson(const Model3D& m, bool pretty = false);
bool fromCustomJson(const std::string& s, Model3D& out, std::string* err = nullptr);

// glTF 2.0 binary (.glb): single-file binary container that Blender opens
// natively (File → Import → glTF 2.0). Encodes sections as cuboid meshes
// and plates as colored quads. Returns the binary as a std::string blob.
// `plate_color_rgb` is the [0..1]³ display color (defaults to magenta).
std::string toGlbBinary(const Model3D& m,
                        const std::array<float, 3>& plate_color_rgb = {0.55f, 0.10f, 0.85f});

// OBJ + MTL fallback (also Blender-native). Returns the two file contents
// as a pair {obj_text, mtl_text}.
std::pair<std::string, std::string>
toObjMtl(const Model3D& m,
         const std::array<float, 3>& plate_color_rgb = {0.55f, 0.10f, 0.85f});

}  // namespace mate
