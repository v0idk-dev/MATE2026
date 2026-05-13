// vision_rectangles.mm — Apple Vision implementation of the colour-agnostic
// rectangle detector declared in include/vision_rectangles.hpp.
//
// Uses VNDetectRectanglesRequest which runs on the Neural Engine and
// returns up to N quadrilateral observations with corner points in
// normalised image coordinates (origin = bottom-left, Y up). We flip Y,
// scale to pixels, and refine corners with cv::cornerSubPix.
//
// Linked frameworks: Vision, CoreImage, CoreVideo, AppKit (already in
// Makefile's Darwin LDFLAGS block).

#import <Foundation/Foundation.h>
#import <Vision/Vision.h>
#import <CoreImage/CoreImage.h>
#import <CoreVideo/CoreVideo.h>
#import <AppKit/AppKit.h>

#include "vision_rectangles.hpp"
#include <opencv2/imgproc.hpp>
#include <vector>

namespace mate {

static CVPixelBufferRef bgrToPixelBuffer(const cv::Mat& bgr) {
    cv::Mat bgra; cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
    CVPixelBufferRef buf = nullptr;
    NSDictionary* attrs = @{(id)kCVPixelBufferIOSurfacePropertiesKey : @{}};
    CVPixelBufferCreate(kCFAllocatorDefault, bgra.cols, bgra.rows,
                        kCVPixelFormatType_32BGRA, (__bridge CFDictionaryRef)attrs, &buf);
    if (!buf) return nullptr;
    CVPixelBufferLockBaseAddress(buf, 0);
    uint8_t* dst = (uint8_t*)CVPixelBufferGetBaseAddress(buf);
    size_t   stride = CVPixelBufferGetBytesPerRow(buf);
    for (int y = 0; y < bgra.rows; ++y)
        memcpy(dst + y * stride, bgra.ptr(y), bgra.cols * 4);
    CVPixelBufferUnlockBaseAddress(buf, 0);
    return buf;
}

std::vector<PlateDetection>
visionDetectRectangles(const cv::Mat& bgr, const VisionRectConfig& cfg) {
    std::vector<PlateDetection> out;
    if (bgr.empty()) return out;

    @autoreleasepool {
        CVPixelBufferRef pb = bgrToPixelBuffer(bgr);
        if (!pb) return out;

        VNImageRequestHandler* handler =
            [[VNImageRequestHandler alloc] initWithCVPixelBuffer:pb options:@{}];

        VNDetectRectanglesRequest* req = [[VNDetectRectanglesRequest alloc] init];
        req.minimumAspectRatio  = cfg.min_aspect_ratio;
        req.maximumAspectRatio  = cfg.max_aspect_ratio;
        req.minimumSize         = (float)(cfg.min_size_px /
                                          (double)std::max(bgr.cols, bgr.rows));
        req.minimumConfidence   = cfg.min_confidence;
        req.maximumObservations = cfg.max_observations;
        req.quadratureTolerance = cfg.quad_tolerance;

        NSError* err = nil;
        [handler performRequests:@[req] error:&err];
        CVPixelBufferRelease(pb);
        if (err) return out;

        cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        const double W = bgr.cols, H = bgr.rows;
        int next_id = 0;

        for (VNRectangleObservation* obs in req.results) {
            // Vision returns normalised coords, origin BOTTOM-left, Y up.
            // Convert to pixel coords with origin top-left.
            auto toPx = [&](CGPoint p) {
                return cv::Point2f(static_cast<float>(p.x * W),
                                   static_cast<float>((1.0 - p.y) * H));
            };
            // Rectangle order in Vision: TL, TR, BR, BL but in its flipped
            // coord system. After Y-flip the ordering becomes BL, BR, TR, TL,
            // so we reorder to TL, TR, BR, BL for downstream PnP.
            std::vector<cv::Point2f> pts = {
                toPx(obs.topLeft), toPx(obs.topRight),
                toPx(obs.bottomRight), toPx(obs.bottomLeft)
            };
            cv::cornerSubPix(gray, pts, {5, 5}, {-1, -1},
                {cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01});

            cv::Point2f c(0, 0); for (auto& p : pts) c += p; c *= 0.25f;
            // Approx area
            double a = std::abs((pts[2].x - pts[0].x) * (pts[2].y - pts[0].y));

            PlateDetection d;
            d.id          = next_id++;
            d.area_px     = a;
            d.confidence  = obs.confidence;
            d.center      = c;
            // PlateDetection::corners is std::array<cv::Point2f,4>
            // (TL, TR, BR, BL). cornerSubPix gave us pts in the same
            // order; copy element-wise.
            d.corners[0] = pts[0]; d.corners[1] = pts[1];
            d.corners[2] = pts[2]; d.corners[3] = pts[3];
            out.push_back(std::move(d));
        }
    }
    return out;
}

std::vector<PlateDetection>
visionDetectRectangles(const cv::Mat& bgr, double plate_side_m,
                       double subject_distance_m, double focal_px) {
    // s_px = f · s_m / Z   (subject distance Z, NOT baseline)
    VisionRectConfig c;
    if (subject_distance_m > 0 && focal_px > 0 && plate_side_m > 0) {
        double exp_px = focal_px * plate_side_m / subject_distance_m;
        c.min_size_px = std::max(8.0, exp_px * 0.4);
    }
    return visionDetectRectangles(bgr, c);
}

}  // namespace mate
