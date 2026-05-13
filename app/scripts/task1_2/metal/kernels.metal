// kernels.metal — Metal shading language compute kernels for image ops.
// Compiled at build time into task1_2_kernels.metallib (see Makefile).
#include <metal_stdlib>
using namespace metal;

// Bilinear remap from {mapx, mapy} (CV_32F) into a CV_8UC3 destination.
// src/dst are flat row-major BGR byte buffers of the same dimensions.
struct RemapParams { int w; int h; };

kernel void k_remap_bilinear_bgr(
    device const uchar*  src   [[buffer(0)]],
    device const float*  mapx  [[buffer(1)]],
    device const float*  mapy  [[buffer(2)]],
    device       uchar*  dst   [[buffer(3)]],
    constant RemapParams& P    [[buffer(4)]],
    uint2 gid [[thread_position_in_grid]])
{
    if ((int)gid.x >= P.w || (int)gid.y >= P.h) return;
    const int idx = gid.y * P.w + gid.x;
    float fx = mapx[idx];
    float fy = mapy[idx];
    if (fx < 0 || fy < 0 || fx > P.w-1 || fy > P.h-1) {
        dst[idx*3+0] = dst[idx*3+1] = dst[idx*3+2] = 0; return;
    }
    int x0 = (int)floor(fx), y0 = (int)floor(fy);
    int x1 = min(x0+1, P.w-1), y1 = min(y0+1, P.h-1);
    float ax = fx - x0, ay = fy - y0;
    for (int c = 0; c < 3; ++c) {
        float v00 = src[(y0*P.w + x0)*3 + c];
        float v01 = src[(y0*P.w + x1)*3 + c];
        float v10 = src[(y1*P.w + x0)*3 + c];
        float v11 = src[(y1*P.w + x1)*3 + c];
        float v   = (1-ax)*(1-ay)*v00 + ax*(1-ay)*v01 + (1-ax)*ay*v10 + ax*ay*v11;
        dst[idx*3 + c] = (uchar)clamp(v, 0.0f, 255.0f);
    }
}

// HSV threshold (H 0..180 OpenCV convention).
struct HsvParams { int w; int h; int h_lo; int h_hi; int s_min; int v_min; int v_max; };

inline float3 bgr2hsv(float3 bgr) {
    float r = bgr.z / 255.0f, g = bgr.y / 255.0f, b = bgr.x / 255.0f;
    float mx = max(r, max(g, b));
    float mn = min(r, min(g, b));
    float v  = mx;
    float s  = (mx <= 1e-6f) ? 0.0f : (mx - mn) / mx;
    float h;
    if (mx == mn)      h = 0;
    else if (mx == r)  h = 60.0f * (g - b) / (mx - mn);
    else if (mx == g)  h = 60.0f * (2.0f + (b - r) / (mx - mn));
    else               h = 60.0f * (4.0f + (r - g) / (mx - mn));
    if (h < 0) h += 360.0f;
    return float3(h * 0.5f, s * 255.0f, v * 255.0f);  // OpenCV scaling
}

kernel void k_hsv_threshold(
    device const uchar* src [[buffer(0)]],
    device       uchar* dst [[buffer(1)]],
    constant HsvParams& P  [[buffer(2)]],
    uint2 gid [[thread_position_in_grid]])
{
    if ((int)gid.x >= P.w || (int)gid.y >= P.h) return;
    int idx = gid.y * P.w + gid.x;
    float3 bgr = float3(src[idx*3+0], src[idx*3+1], src[idx*3+2]);
    float3 hsv = bgr2hsv(bgr);
    int h = (int)hsv.x;
    bool ok_h = (P.h_lo <= P.h_hi) ? (h >= P.h_lo && h <= P.h_hi)
                                   : (h >= P.h_lo || h <= P.h_hi);
    bool ok_s = hsv.y >= P.s_min;
    bool ok_v = hsv.z >= P.v_min && hsv.z <= P.v_max;
    dst[idx] = (ok_h && ok_s && ok_v) ? 255 : 0;
}

// "Canny lite": gaussian → Sobel magnitude → simple threshold.
// Real non-max suppression and hysteresis are skipped here for simplicity;
// the result is good enough for our pipe-edge step which post-filters.
struct EdgeParams { int w; int h; int lo; int hi; int r; };

kernel void k_canny_lite(
    device const uchar* src [[buffer(0)]],
    device       uchar* dst [[buffer(1)]],
    constant EdgeParams& P [[buffer(2)]],
    uint2 gid [[thread_position_in_grid]])
{
    if ((int)gid.x >= P.w || (int)gid.y >= P.h) return;
    int idx = gid.y * P.w + gid.x;
    if (gid.x < 1 || gid.y < 1 || (int)gid.x >= P.w-1 || (int)gid.y >= P.h-1) {
        dst[idx] = 0; return;
    }
    auto lum = [&](int x, int y) {
        int k = (y * P.w + x) * 3;
        return (float)(0.114f*src[k+0] + 0.587f*src[k+1] + 0.299f*src[k+2]);
    };
    float gx = -lum(gid.x-1, gid.y-1) -2*lum(gid.x-1, gid.y) - lum(gid.x-1, gid.y+1)
             + lum(gid.x+1, gid.y-1) +2*lum(gid.x+1, gid.y) + lum(gid.x+1, gid.y+1);
    float gy = -lum(gid.x-1, gid.y-1) -2*lum(gid.x, gid.y-1) - lum(gid.x+1, gid.y-1)
             + lum(gid.x-1, gid.y+1) +2*lum(gid.x, gid.y+1) + lum(gid.x+1, gid.y+1);
    float mag = sqrt(gx*gx + gy*gy);
    dst[idx] = (mag >= P.hi) ? 255 : ((mag >= P.lo) ? 128 : 0);
}

// 3x3 median for CV_32F.
struct MedParams { int w; int h; };

kernel void k_median3_f32(
    device const float* src [[buffer(0)]],
    device       float* dst [[buffer(1)]],
    constant MedParams& P [[buffer(2)]],
    uint2 gid [[thread_position_in_grid]])
{
    if ((int)gid.x >= P.w || (int)gid.y >= P.h) return;
    int idx = gid.y * P.w + gid.x;
    if (gid.x < 1 || gid.y < 1 || (int)gid.x >= P.w-1 || (int)gid.y >= P.h-1) {
        dst[idx] = src[idx]; return;
    }
    float v[9];
    int k = 0;
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx)
            v[k++] = src[(gid.y+dy)*P.w + (gid.x+dx)];
    // Insertion sort 9 elements.
    for (int i = 1; i < 9; ++i) {
        float x = v[i]; int j = i - 1;
        while (j >= 0 && v[j] > x) { v[j+1] = v[j]; --j; }
        v[j+1] = x;
    }
    dst[idx] = v[4];
}
