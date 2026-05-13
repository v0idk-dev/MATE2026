// model3d.cpp — implementation of the canonical model + JSON / GLB / OBJ I/O.
#include "model3d.hpp"
#include "../json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <vector>

namespace mate {

namespace {

// Face → (origin offset from section center, tangent_u, tangent_v).
// Section center is `origin` (which is the center of the bottom face)
// shifted by (0, 0, h/2). All in section-local axes.
struct FaceFrame {
    Vec3 center_offset;   // from section center to face center
    Vec3 tu, tv;           // unit tangents (u along, v along)
    Vec3 n;                // outward normal
    Vec3 size;             // (extent_u, extent_v) baked as (x,y, ignore)
};

FaceFrame faceFrame(const Section& s, const std::string& face) {
    const double L = s.size.x, W = s.size.y, H = s.size.z;
    if (face == "+x") return {{ +L/2, 0, 0}, {0,1,0}, {0,0,1}, {1,0,0}, {W,H,0}};
    if (face == "-x") return {{ -L/2, 0, 0}, {0,1,0}, {0,0,1}, {-1,0,0},{W,H,0}};
    if (face == "+y") return {{0, +W/2, 0}, {1,0,0}, {0,0,1}, {0,1,0}, {L,H,0}};
    if (face == "-y") return {{0, -W/2, 0}, {1,0,0}, {0,0,1}, {0,-1,0},{L,H,0}};
    if (face == "-z") return {{0, 0, -H/2}, {1,0,0}, {0,1,0}, {0,0,-1},{L,W,0}};
    return         { {0, 0, +H/2}, {1,0,0}, {0,1,0}, {0,0,1}, {L,W,0}}; // +z (default)
}

Vec3 rotZ(const Vec3& p, double yaw_rad) {
    const double c = std::cos(yaw_rad), s = std::sin(yaw_rad);
    return { c*p.x - s*p.y, s*p.x + c*p.y, p.z };
}

// Resolve world-space corners of one plate. Order: TL, TR, BR, BL with
// respect to (tu, tv) on the named face.
std::array<Vec3,4> resolveOne(const Section& s, const Plate& p) {
    FaceFrame ff = faceFrame(s, p.face);
    // Section center in world frame:
    Vec3 sc = { s.origin.x, s.origin.y, s.origin.z + s.size.z/2.0 };
    // Face center in section-local frame, then world frame (after yaw):
    Vec3 fc_local = ff.center_offset;
    Vec3 fc_world_section = rotZ(fc_local, s.yaw_deg * M_PI/180.0);
    Vec3 fc = { sc.x + fc_world_section.x, sc.y + fc_world_section.y, sc.z + fc_world_section.z };
    Vec3 tu = rotZ(ff.tu, s.yaw_deg * M_PI/180.0);
    Vec3 tv = rotZ(ff.tv, s.yaw_deg * M_PI/180.0);
    // Plate center on the face from (u,v):
    const double eu = ff.size.x, ev = ff.size.y;
    const double cu = (p.u - 0.5) * eu;
    const double cv = (p.v - 0.5) * ev;
    Vec3 pc = { fc.x + tu.x*cu + tv.x*cv,
                fc.y + tu.y*cu + tv.y*cv,
                fc.z + tu.z*cu + tv.z*cv };
    const double half = p.side_m * 0.5;
    auto add = [&](double a, double b) {
        return Vec3{ pc.x + tu.x*a + tv.x*b,
                     pc.y + tu.y*a + tv.y*b,
                     pc.z + tu.z*a + tv.z*b };
    };
    return { add(-half, +half),  // TL
             add(+half, +half),  // TR
             add(+half, -half),  // BR
             add(-half, -half) };// BL
}

}  // namespace

void resolvePlateCorners(Model3D& m) {
    for (auto& p : m.plates) {
        if (p.corners_resolved) continue;
        if (p.section_id < 0 || p.section_id >= (int)m.sections.size()) continue;
        p.corners = resolveOne(m.sections[p.section_id], p);
        p.corners_resolved = true;
    }
}

void recomputeBounds(Model3D& m) {
    if (m.sections.empty()) {
        m.total_length = m.total_width = m.total_height = 0.0;
        return;
    }
    double xmin=1e30, xmax=-1e30, ymin=1e30, ymax=-1e30, zmin=1e30, zmax=-1e30;
    for (const auto& s : m.sections) {
        const double hx=s.size.x*0.5, hy=s.size.y*0.5;
        const double cz = s.origin.z, tz = s.origin.z + s.size.z;
        // Account for yaw by checking 4 footprint corners.
        for (int i=0;i<4;i++) {
            const double sx = (i&1)? hx : -hx;
            const double sy = (i&2)? hy : -hy;
            Vec3 r = rotZ({sx,sy,0}, s.yaw_deg * M_PI/180.0);
            const double x = s.origin.x + r.x, y = s.origin.y + r.y;
            xmin=std::min(xmin,x); xmax=std::max(xmax,x);
            ymin=std::min(ymin,y); ymax=std::max(ymax,y);
        }
        zmin = std::min(zmin, cz);
        zmax = std::max(zmax, tz);
    }
    m.total_length = xmax - xmin;
    m.total_width  = ymax - ymin;
    m.total_height = zmax - zmin;
}

void applyScale(Model3D& m, double k, const std::string& source,
                const std::string& reason, double confidence) {
    if (k <= 0 || !std::isfinite(k)) return;
    for (auto& s : m.sections) {
        s.origin.x *= k; s.origin.y *= k; s.origin.z *= k;
        s.size.x   *= k; s.size.y   *= k; s.size.z   *= k;
    }
    for (auto& p : m.plates) {
        // side_m is a known physical constant (10 cm) — never scaled
        if (p.corners_resolved)
            for (auto& c : p.corners) { c.x*=k; c.y*=k; c.z*=k; }
    }
    for (auto& v : m.raw_plate_centers) { v.x*=k; v.y*=k; v.z*=k; }
    // total_width is the known spec dimension — never scaled
    m.total_length *= k; m.total_height *= k;
    m.scale.k *= k;
    m.scale.source = source;
    m.scale.reason = reason;
    m.scale.confidence = confidence;
}

// ── Custom JSON ──────────────────────────────────────────────────────────
using nlohmann::json;

static json v3(const Vec3& v){ return json::array({v.x, v.y, v.z}); }
static Vec3 v3r(const json& j){
    if (!j.is_array() || j.size() < 3) return {};
    return { j[0].get<double>(), j[1].get<double>(), j[2].get<double>() };
}

std::string toCustomJson(const Model3D& m, bool pretty) {
    json j;
    j["schema"]  = "mate.coral_garden.v1";
    j["version"] = m.version;
    j["unit"]    = m.unit;

    json js = json::array();
    for (const auto& s : m.sections) {
        js.push_back({
            {"id", s.id},
            {"origin", v3(s.origin)},
            {"size",   v3(s.size)},
            {"yaw_deg",s.yaw_deg},
            {"confidence", s.confidence}});
    }
    j["sections"] = js;

    json jp = json::array();
    for (const auto& p : m.plates) {
        json one = {
            {"id", p.id},
            {"section_id", p.section_id},
            {"face", p.face},
            {"u", p.u}, {"v", p.v},
            {"side_m", p.side_m},
            {"confidence", p.confidence}};
        if (p.corners_resolved) {
            json cj = json::array();
            for (const auto& c : p.corners) cj.push_back(v3(c));
            one["corners"] = cj;
        }
        jp.push_back(one);
    }
    j["plates"] = jp;

    j["totals"] = {
        {"length", m.total_length},
        {"width",  m.total_width},
        {"height", m.total_height}};

    j["scale"] = {
        {"k", m.scale.k},
        {"source", m.scale.source},
        {"confidence", m.scale.confidence},
        {"reason", m.scale.reason}};

    j["calibration"] = {
        {"present", m.calibration.present},
        {"rms_px",  m.calibration.rms_px},
        {"avg_epipolar_err_px", m.calibration.avg_epipolar_err_px},
        {"baseline_m", m.calibration.baseline_m},
        {"image_width",  m.calibration.image_width},
        {"image_height", m.calibration.image_height},
        {"pairs_used",   m.calibration.pairs_used}};

    j["n_pairs_used"] = m.n_pairs_used;
    j["timing_ms"]    = m.timing_ms;
    if (!m.warning.empty()) j["warning"] = m.warning;

    return pretty ? j.dump(2) : j.dump();
}

bool fromCustomJson(const std::string& s, Model3D& out, std::string* err) {
    try {
        json j = json::parse(s);
        out = {};
        out.version = j.value("version", 1);
        out.unit    = j.value("unit", std::string("m"));

        if (j.contains("sections"))
            for (const auto& sj : j["sections"]) {
                Section s2;
                s2.id        = sj.value("id", -1);
                s2.origin    = v3r(sj.value("origin", json::array({0,0,0})));
                s2.size      = v3r(sj.value("size",   json::array({0,0,0})));
                s2.yaw_deg   = sj.value("yaw_deg", 0.0);
                s2.confidence= sj.value("confidence", 0.0);
                out.sections.push_back(s2);
            }
        if (j.contains("plates"))
            for (const auto& pj : j["plates"]) {
                Plate p;
                p.id         = pj.value("id", -1);
                p.section_id = pj.value("section_id", -1);
                p.face       = pj.value("face", std::string("+z"));
                p.u          = pj.value("u", 0.5);
                p.v          = pj.value("v", 0.5);
                p.side_m     = pj.value("side_m", 0.10);
                p.confidence = pj.value("confidence", 0.0);
                if (pj.contains("corners")) {
                    int i = 0;
                    for (const auto& cj : pj["corners"]) {
                        if (i >= 4) break;
                        p.corners[i++] = v3r(cj);
                    }
                    p.corners_resolved = (i == 4);
                }
                out.plates.push_back(p);
            }
        if (j.contains("totals")) {
            out.total_length = j["totals"].value("length", 0.0);
            out.total_width  = j["totals"].value("width",  0.0);
            out.total_height = j["totals"].value("height", 0.0);
        }
        if (j.contains("scale")) {
            out.scale.k          = j["scale"].value("k", 1.0);
            out.scale.source     = j["scale"].value("source", "");
            out.scale.confidence = j["scale"].value("confidence", 0.0);
            out.scale.reason     = j["scale"].value("reason", "");
        }
        if (j.contains("calibration")) {
            const auto& c = j["calibration"];
            out.calibration.present              = c.value("present", false);
            out.calibration.rms_px               = c.value("rms_px", -1.0);
            out.calibration.avg_epipolar_err_px  = c.value("avg_epipolar_err_px", -1.0);
            out.calibration.baseline_m           = c.value("baseline_m", 0.0);
            out.calibration.image_width          = c.value("image_width", 0);
            out.calibration.image_height         = c.value("image_height", 0);
            out.calibration.pairs_used           = c.value("pairs_used", 0);
        }
        out.n_pairs_used = j.value("n_pairs_used", 0);
        out.timing_ms    = j.value("timing_ms", 0LL);
        out.warning      = j.value("warning", std::string(""));
        return true;
    } catch (const std::exception& e) {
        if (err) *err = e.what();
        return false;
    }
}

// ── GLB writer ───────────────────────────────────────────────────────────
//
// Minimal hand-rolled glTF 2.0 binary. We emit:
//   • One mesh per Section (cuboid, 24 verts / 36 indices, w/ normals).
//   • One mesh per Plate (4 verts / 6 indices, single-color material).
// Two materials: white-ish for sections, plate_color_rgb for plates.
//
// Layout in the binary buffer:
//   [section_positions][section_normals][section_indices][plate_positions][plate_indices]
//
namespace {

void pushFloat3(std::vector<uint8_t>& bin, float a, float b, float c) {
    auto p = (uint8_t*)&a; bin.insert(bin.end(), p, p+4);
         p = (uint8_t*)&b; bin.insert(bin.end(), p, p+4);
         p = (uint8_t*)&c; bin.insert(bin.end(), p, p+4);
}
void pushU16(std::vector<uint8_t>& bin, uint16_t x) {
    auto p = (uint8_t*)&x; bin.insert(bin.end(), p, p+2);
}
void padTo4(std::vector<uint8_t>& v, uint8_t pad = 0) {
    while (v.size() % 4) v.push_back(pad);
}

// Generate cuboid corners (8) in world coords for one section, applying yaw.
std::array<Vec3,8> sectionBoxCorners(const Section& s) {
    const double hx=s.size.x*0.5, hy=s.size.y*0.5, hz=s.size.z*0.5;
    Vec3 c = { s.origin.x, s.origin.y, s.origin.z + hz };
    std::array<Vec3,8> out;
    for (int i=0;i<8;i++) {
        const double sx = (i&1)? hx : -hx;
        const double sy = (i&2)? hy : -hy;
        const double sz = (i&4)? hz : -hz;
        Vec3 r = rotZ({sx,sy,sz}, s.yaw_deg * M_PI/180.0);
        out[i] = { c.x + r.x, c.y + r.y, c.z + r.z };
    }
    return out;
}

}  // namespace

std::string toGlbBinary(const Model3D& m_in, const std::array<float,3>& plate_rgb) {
    Model3D m = m_in;
    resolvePlateCorners(m);

    // Build BIN chunk.
    std::vector<uint8_t> bin;

    // (1) Section positions — 24 unique verts per section (per-face normals).
    const size_t section_pos_off = bin.size();
    // 6 faces × 4 verts × 3 floats = 72 floats per section.
    std::vector<std::array<int,8>> idx; // store cuboid corner indices for face vertex emission
    for (const auto& s : m.sections) {
        auto C = sectionBoxCorners(s);
        // Face vertex index sets, each 4 corner indices CCW from outside.
        const int faces[6][4] = {
            {1,3,7,5}, // +x
            {2,0,4,6}, // -x
            {3,2,6,7}, // +y
            {0,1,5,4}, // -y
            {4,5,7,6}, // +z (top)
            {2,3,1,0}  // -z (bottom)
        };
        for (int f=0; f<6; ++f)
            for (int k=0; k<4; ++k) {
                const auto& v = C[faces[f][k]];
                pushFloat3(bin, (float)v.x, (float)v.y, (float)v.z);
            }
    }
    const size_t section_pos_len = bin.size() - section_pos_off;

    // (2) Section normals — same count as positions.
    const size_t section_norm_off = bin.size();
    {
        const float N[6][3] = {
            { 1, 0, 0}, {-1, 0, 0}, { 0, 1, 0}, { 0,-1, 0}, { 0, 0, 1}, { 0, 0,-1}};
        for (size_t s=0; s<m.sections.size(); ++s)
            for (int f=0; f<6; ++f)
                for (int k=0; k<4; ++k)
                    pushFloat3(bin, N[f][0], N[f][1], N[f][2]);
    }
    const size_t section_norm_len = bin.size() - section_norm_off;

    // (3) Section indices — 6 faces × 6 indices per face (two triangles).
    // Indices are LOCAL (0-based within each section's 24 verts) so each
    // section accessor can reference its own 24-vertex slice of the position
    // bufferView without the viewer going out of bounds.
    const size_t section_idx_off = bin.size();
    for (size_t s=0; s<m.sections.size(); ++s) {
        for (int f=0; f<6; ++f) {
            const uint16_t b = (uint16_t)(f*4);  // local 0..23
            pushU16(bin, b+0); pushU16(bin, b+1); pushU16(bin, b+2);
            pushU16(bin, b+0); pushU16(bin, b+2); pushU16(bin, b+3);
        }
    }
    const size_t section_idx_len = bin.size() - section_idx_off;
    padTo4(bin);

    // (4) Plate positions: 4 verts per plate.
    const size_t plate_pos_off = bin.size();
    for (const auto& p : m.plates) {
        if (!p.corners_resolved) { for (int k=0;k<4;k++) pushFloat3(bin,0,0,0); continue; }
        for (int k=0; k<4; ++k)
            pushFloat3(bin, (float)p.corners[k].x, (float)p.corners[k].y, (float)p.corners[k].z);
    }
    const size_t plate_pos_len = bin.size() - plate_pos_off;

    // (5) Plate indices: 6 per plate. Local 0-based so each plate accessor
    // (count=4) maps correctly to its own slice of the position bufferView.
    const size_t plate_idx_off = bin.size();
    for (size_t i=0; i<m.plates.size(); ++i) {
        pushU16(bin, 0); pushU16(bin, 1); pushU16(bin, 2);
        pushU16(bin, 0); pushU16(bin, 2); pushU16(bin, 3);
    }
    const size_t plate_idx_len = bin.size() - plate_idx_off;
    padTo4(bin);

    // Build JSON chunk.
    json j;
    j["asset"] = {{"version", "2.0"}, {"generator", "MATE coral_garden v1"}};
    j["scenes"] = json::array({{{"nodes", json::array()}}});
    j["scene"]  = 0;
    j["nodes"]  = json::array();
    j["meshes"] = json::array();
    j["bufferViews"] = json::array();
    j["accessors"]   = json::array();

    // Materials: 0 = section (light gray), 1 = plate (configurable).
    j["materials"] = json::array({
        json{{"name","section"},
             {"pbrMetallicRoughness", {
                 {"baseColorFactor", json::array({0.85f, 0.85f, 0.88f, 1.0f})},
                 {"metallicFactor", 0.0f},
                 {"roughnessFactor", 0.85f}}},
             {"doubleSided", true}},
        json{{"name","plate"},
             {"pbrMetallicRoughness", {
                 {"baseColorFactor", json::array({plate_rgb[0], plate_rgb[1], plate_rgb[2], 1.0f})},
                 {"metallicFactor", 0.0f},
                 {"roughnessFactor", 0.5f}}},
             {"doubleSided", true}}});

    auto addBV = [&](size_t off, size_t len, int target = 0) -> int {
        json bv = {{"buffer",0},{"byteOffset", off},{"byteLength", len}};
        if (target) bv["target"] = target;
        j["bufferViews"].push_back(bv);
        return (int)j["bufferViews"].size() - 1;
    };
    [[maybe_unused]] auto addAccF3 = [&](int bv, size_t count, const std::array<float,3>& mn, const std::array<float,3>& mx) {
        j["accessors"].push_back({
            {"bufferView", bv}, {"componentType", 5126}, {"count", count},
            {"type", "VEC3"},
            {"min", json::array({mn[0],mn[1],mn[2]})},
            {"max", json::array({mx[0],mx[1],mx[2]})}});
        return (int)j["accessors"].size() - 1;
    };
    [[maybe_unused]] auto addAccU16 = [&](int bv, size_t count) {
        j["accessors"].push_back({
            {"bufferView", bv}, {"componentType", 5123}, {"count", count}, {"type","SCALAR"}});
        return (int)j["accessors"].size() - 1;
    };

    // Per-section bounds for accessor min/max (required for POSITION).
    auto bv_pos = addBV(section_pos_off, section_pos_len, 34962);
    auto bv_nrm = addBV(section_norm_off, section_norm_len, 34962);
    auto bv_idx = addBV(section_idx_off, section_idx_len, 34963);

    // For each section: one mesh with one primitive, slicing into the shared buffers.
    for (size_t s=0; s<m.sections.size(); ++s) {
        // Compute min/max for this section's 24 positions.
        float mn[3] = { 1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f,-1e30f,-1e30f};
        const float* pos = (const float*)&bin[section_pos_off + s*24*3*sizeof(float)];
        for (int v=0; v<24; ++v) {
            for (int c=0; c<3; ++c) {
                mn[c] = std::min(mn[c], pos[v*3+c]);
                mx[c] = std::max(mx[c], pos[v*3+c]);
            }
        }
        // Sub-bufferViews per section (offset into the parent positions BV by byteOffset on accessor):
        int acc_pos = (int)j["accessors"].size();
        j["accessors"].push_back({
            {"bufferView", bv_pos},
            {"byteOffset", s*24*3*sizeof(float)},
            {"componentType", 5126},
            {"count", 24}, {"type","VEC3"},
            {"min", json::array({mn[0],mn[1],mn[2]})},
            {"max", json::array({mx[0],mx[1],mx[2]})}});
        int acc_nrm = (int)j["accessors"].size();
        j["accessors"].push_back({
            {"bufferView", bv_nrm},
            {"byteOffset", s*24*3*sizeof(float)},
            {"componentType", 5126},
            {"count", 24}, {"type","VEC3"}});
        int acc_idx = (int)j["accessors"].size();
        j["accessors"].push_back({
            {"bufferView", bv_idx},
            {"byteOffset", s*36*sizeof(uint16_t)},
            {"componentType", 5123},
            {"count", 36}, {"type","SCALAR"}});

        json mesh = {
            {"name", std::string("section_") + std::to_string(m.sections[s].id)},
            {"primitives", json::array({
                json{{"attributes", {{"POSITION", acc_pos}, {"NORMAL", acc_nrm}}},
                     {"indices", acc_idx},
                     {"material", 0}}})}};
        j["meshes"].push_back(mesh);
        j["nodes"].push_back({{"mesh", j["meshes"].size()-1}});
        j["scenes"][0]["nodes"].push_back(j["nodes"].size()-1);
    }

    // Plate buffer views & meshes.
    if (!m.plates.empty()) {
        auto bv_ppos = addBV(plate_pos_off, plate_pos_len, 34962);
        auto bv_pidx = addBV(plate_idx_off, plate_idx_len, 34963);
        for (size_t i=0; i<m.plates.size(); ++i) {
            float mn[3] = { 1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f,-1e30f,-1e30f};
            const float* pos = (const float*)&bin[plate_pos_off + i*4*3*sizeof(float)];
            for (int v=0; v<4; ++v)
                for (int c=0; c<3; ++c) {
                    mn[c] = std::min(mn[c], pos[v*3+c]);
                    mx[c] = std::max(mx[c], pos[v*3+c]);
                }
            int acc_pos = (int)j["accessors"].size();
            j["accessors"].push_back({
                {"bufferView", bv_ppos},
                {"byteOffset", i*4*3*sizeof(float)},
                {"componentType", 5126},
                {"count", 4}, {"type","VEC3"},
                {"min", json::array({mn[0],mn[1],mn[2]})},
                {"max", json::array({mx[0],mx[1],mx[2]})}});
            int acc_idx = (int)j["accessors"].size();
            j["accessors"].push_back({
                {"bufferView", bv_pidx},
                {"byteOffset", i*6*sizeof(uint16_t)},
                {"componentType", 5123},
                {"count", 6}, {"type","SCALAR"}});

            json mesh = {
                {"name", std::string("plate_") + std::to_string(m.plates[i].id)},
                {"primitives", json::array({
                    json{{"attributes", {{"POSITION", acc_pos}}},
                         {"indices", acc_idx},
                         {"material", 1}}})}};
            j["meshes"].push_back(mesh);
            j["nodes"].push_back({{"mesh", j["meshes"].size()-1}});
            j["scenes"][0]["nodes"].push_back(j["nodes"].size()-1);
        }
    }

    j["buffers"] = json::array({{{"byteLength", bin.size()}}});
    std::string js = j.dump();
    while (js.size() % 4) js.push_back(' ');

    // Assemble GLB: 12-byte header + JSON chunk + BIN chunk.
    std::string out;
    auto put32 = [&](uint32_t v){ out.append((char*)&v, 4); };
    put32(0x46546C67);                      // "glTF"
    put32(2);                               // version 2
    put32(12 + 8 + (uint32_t)js.size() + 8 + (uint32_t)bin.size());
    put32((uint32_t)js.size());             // JSON chunk length
    put32(0x4E4F534A);                      // "JSON"
    out.append(js);
    put32((uint32_t)bin.size());            // BIN chunk length
    put32(0x004E4942);                      // "BIN\0"
    out.append((char*)bin.data(), bin.size());
    return out;
}

// ── OBJ + MTL writer ─────────────────────────────────────────────────────
std::pair<std::string,std::string>
toObjMtl(const Model3D& m_in, const std::array<float,3>& plate_rgb) {
    Model3D m = m_in;
    resolvePlateCorners(m);

    std::ostringstream obj, mtl;
    mtl << "newmtl section\nKd 0.85 0.85 0.88\nKa 0 0 0\nKs 0 0 0\nNs 30\nillum 2\n\n"
        << "newmtl plate\nKd " << plate_rgb[0] << " " << plate_rgb[1] << " " << plate_rgb[2]
        << "\nKa 0 0 0\nKs 0 0 0\nNs 30\nillum 2\n";

    obj << "# MATE coral_garden v1\nmtllib model.mtl\n";
    int vi = 1; // 1-based OBJ vertex index

    for (const auto& s : m.sections) {
        auto C = sectionBoxCorners(s);
        obj << "o section_" << s.id << "\n";
        for (auto& v : C) obj << "v " << v.x << " " << v.y << " " << v.z << "\n";
        obj << "usemtl section\n";
        const int b = vi;
        // 12 triangles, 6 quads:
        const int F[6][4] = {
            {1,3,7,5}, {2,0,4,6}, {3,2,6,7}, {0,1,5,4}, {4,5,7,6}, {2,3,1,0} };
        for (int f=0; f<6; ++f) {
            int a=F[f][0]+b, c=F[f][1]+b, d=F[f][2]+b, e=F[f][3]+b;
            obj << "f " << a << " " << c << " " << d << "\n";
            obj << "f " << a << " " << d << " " << e << "\n";
        }
        vi += 8;
    }
    for (const auto& p : m.plates) {
        if (!p.corners_resolved) continue;
        obj << "o plate_" << p.id << "\n";
        for (const auto& c : p.corners) obj << "v " << c.x << " " << c.y << " " << c.z << "\n";
        obj << "usemtl plate\n";
        obj << "f " << vi << " " << vi+1 << " " << vi+2 << "\n";
        obj << "f " << vi << " " << vi+2 << " " << vi+3 << "\n";
        vi += 4;
    }
    return { obj.str(), mtl.str() };
}

}  // namespace mate
