// metal_compute.mm — Metal-backed implementations of mate::metal::*.
//
// Apple Silicon only. Compiled with `clang++ -ObjC++ -fobjc-arc`. The
// Metal shading language source for the kernels lives in
// scripts/task1_2/metal/kernels.metal — at build time it is compiled into
// a `.metallib` and loaded at runtime via newDefaultLibrary.
//
// On non-Apple builds this file is not compiled; image_undistort.cpp etc.
// fall through to OpenCV.
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <CoreVideo/CoreVideo.h>
#include "metal_compute.hpp"
#include <opencv2/imgproc.hpp>
#include <mutex>

namespace mate::metal {

namespace {

id<MTLDevice>           gDevice  = nil;
id<MTLCommandQueue>     gQueue   = nil;
id<MTLLibrary>          gLib     = nil;
id<MTLComputePipelineState> gRemap = nil, gHsv = nil, gCanny = nil, gMedF32 = nil;
std::once_flag gInitFlag;
bool gOk = false;

id<MTLComputePipelineState> makePso(id<MTLLibrary> lib, NSString* name) {
    NSError* err = nil;
    id<MTLFunction> f = [lib newFunctionWithName:name];
    if (!f) return nil;
    return [lib.device newComputePipelineStateWithFunction:f error:&err];
}

void doInit() {
    @autoreleasepool {
        gDevice = MTLCreateSystemDefaultDevice();
        if (!gDevice) return;
        gQueue  = [gDevice newCommandQueue];
        NSError* err = nil;
        // Prefer the .metallib next to the binary, fall back to default.
        NSString* libPath = [[NSBundle mainBundle] pathForResource:@"task1_2_kernels" ofType:@"metallib"];
        if (libPath) gLib = [gDevice newLibraryWithFile:libPath error:&err];
        if (!gLib)   gLib = [gDevice newDefaultLibrary];
        if (!gLib)   return;
        gRemap  = makePso(gLib, @"k_remap_bilinear_bgr");
        gHsv    = makePso(gLib, @"k_hsv_threshold");
        gCanny  = makePso(gLib, @"k_canny_lite");
        gMedF32 = makePso(gLib, @"k_median3_f32");
        gOk = (gRemap && gHsv && gCanny && gMedF32);
    }
}

id<MTLBuffer> bufferFromMat(const cv::Mat& m) {
    return [gDevice newBufferWithBytes:m.data
                                length:m.total() * m.elemSize()
                               options:MTLResourceStorageModeShared];
}
cv::Mat matFromBuffer(id<MTLBuffer> buf, int rows, int cols, int type) {
    cv::Mat out(rows, cols, type);
    memcpy(out.data, buf.contents, out.total() * out.elemSize());
    return out;
}

void dispatch1(id<MTLComputePipelineState> pso,
               NSArray<id<MTLBuffer>>* bufs,
               int width, int height) {
    id<MTLCommandBuffer> cb = [gQueue commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    [enc setComputePipelineState:pso];
    NSUInteger i = 0;
    for (id<MTLBuffer> b in bufs) [enc setBuffer:b offset:0 atIndex:i++];
    MTLSize tg = MTLSizeMake(16, 16, 1);
    MTLSize grid = MTLSizeMake((width + 15) / 16 * 16, (height + 15) / 16 * 16, 1);
    [enc dispatchThreads:grid threadsPerThreadgroup:tg];
    [enc endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
}

}  // namespace

bool ensureInitialized() {
    std::call_once(gInitFlag, doInit);
    return gOk;
}

cv::Mat remapBgr(const cv::Mat& src, const cv::Mat& mapx, const cv::Mat& mapy) {
    if (!ensureInitialized()) {
        cv::Mat dst; cv::remap(src, dst, mapx, mapy, cv::INTER_LINEAR); return dst;
    }
    cv::Mat src8 = src; if (src.type() != CV_8UC3) cv::cvtColor(src, src8, cv::COLOR_BGRA2BGR);
    @autoreleasepool {
        id<MTLBuffer> bSrc = bufferFromMat(src8);
        id<MTLBuffer> bMx  = bufferFromMat(mapx);
        id<MTLBuffer> bMy  = bufferFromMat(mapy);
        id<MTLBuffer> bDst = [gDevice newBufferWithLength:src8.total() * 3 options:MTLResourceStorageModeShared];
        struct { int w, h; } params{ src8.cols, src8.rows };
        id<MTLBuffer> bP   = [gDevice newBufferWithBytes:&params length:sizeof(params) options:MTLResourceStorageModeShared];
        dispatch1(gRemap, @[bSrc, bMx, bMy, bDst, bP], src8.cols, src8.rows);
        return matFromBuffer(bDst, src8.rows, src8.cols, CV_8UC3);
    }
}

cv::Mat hsvThreshold(const cv::Mat& src, int h_lo, int h_hi, int s_min, int v_min, int v_max) {
    if (!ensureInitialized()) {
        cv::Mat hsv, dst; cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
        if (h_lo <= h_hi) cv::inRange(hsv, cv::Scalar(h_lo, s_min, v_min), cv::Scalar(h_hi, 255, v_max), dst);
        else { cv::Mat a, b;
            cv::inRange(hsv, cv::Scalar(0, s_min, v_min), cv::Scalar(h_hi, 255, v_max), a);
            cv::inRange(hsv, cv::Scalar(h_lo, s_min, v_min), cv::Scalar(180, 255, v_max), b);
            cv::bitwise_or(a, b, dst);
        } return dst;
    }
    @autoreleasepool {
        id<MTLBuffer> bSrc = bufferFromMat(src);
        id<MTLBuffer> bDst = [gDevice newBufferWithLength:src.total() options:MTLResourceStorageModeShared];
        struct { int w, h, h_lo, h_hi, s_min, v_min, v_max; } p{
            src.cols, src.rows, h_lo, h_hi, s_min, v_min, v_max };
        id<MTLBuffer> bP = [gDevice newBufferWithBytes:&p length:sizeof(p) options:MTLResourceStorageModeShared];
        dispatch1(gHsv, @[bSrc, bDst, bP], src.cols, src.rows);
        return matFromBuffer(bDst, src.rows, src.cols, CV_8UC1);
    }
}

cv::Mat cannyEdges(const cv::Mat& src, int lo, int hi, int radius) {
    if (!ensureInitialized()) {
        cv::Mat g, d; cv::cvtColor(src, g, cv::COLOR_BGR2GRAY);
        if (radius > 0) cv::GaussianBlur(g, g, cv::Size(2*radius+1, 2*radius+1), 0);
        cv::Canny(g, d, lo, hi); return d;
    }
    @autoreleasepool {
        id<MTLBuffer> bSrc = bufferFromMat(src);
        id<MTLBuffer> bDst = [gDevice newBufferWithLength:src.total() options:MTLResourceStorageModeShared];
        struct { int w, h, lo, hi, r; } p{ src.cols, src.rows, lo, hi, radius };
        id<MTLBuffer> bP = [gDevice newBufferWithBytes:&p length:sizeof(p) options:MTLResourceStorageModeShared];
        dispatch1(gCanny, @[bSrc, bDst, bP], src.cols, src.rows);
        return matFromBuffer(bDst, src.rows, src.cols, CV_8UC1);
    }
}

cv::Mat medianFilterF32(const cv::Mat& disp, int win) {
    if (!ensureInitialized() || win != 3) {
        cv::Mat out; cv::medianBlur(disp, out, win); return out;
    }
    @autoreleasepool {
        id<MTLBuffer> bSrc = bufferFromMat(disp);
        id<MTLBuffer> bDst = [gDevice newBufferWithLength:disp.total() * sizeof(float) options:MTLResourceStorageModeShared];
        struct { int w, h; } p{ disp.cols, disp.rows };
        id<MTLBuffer> bP = [gDevice newBufferWithBytes:&p length:sizeof(p) options:MTLResourceStorageModeShared];
        dispatch1(gMedF32, @[bSrc, bDst, bP], disp.cols, disp.rows);
        return matFromBuffer(bDst, disp.rows, disp.cols, CV_32FC1);
    }
}

}  // namespace mate::metal
