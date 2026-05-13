// apple_vision.mm — Apple Vision framework bridge.
// Compiled with -ObjC++ -fobjc-arc.
#import <Vision/Vision.h>
#import <CoreImage/CoreImage.h>
#import <AppKit/AppKit.h>
#include "apple_vision.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace mate::vision {

namespace {

CIImage* matToCIImage(const cv::Mat& bgr) {
    cv::Mat rgba; cv::cvtColor(bgr, rgba, cv::COLOR_BGR2BGRA);
    NSData* d = [NSData dataWithBytes:rgba.data length:rgba.total()*4];
    return [CIImage imageWithBitmapData:d
                              bytesPerRow:rgba.cols*4
                                     size:CGSizeMake(rgba.cols, rgba.rows)
                                   format:kCIFormatBGRA8
                               colorSpace:CGColorSpaceCreateDeviceRGB()];
}

cv::Point2f vnToCv(CGPoint p, int w, int h) {
    // Vision uses normalized coords with (0,0) at bottom-left.
    return cv::Point2f((float)(p.x * w), (float)((1.0 - p.y) * h));
}

}  // namespace

std::vector<Quad> detectRectangles(const cv::Mat& bgr, const cv::Rect& roi,
                                   int max_n, float min_aspect, float min_conf) {
    std::vector<Quad> out;
    if (bgr.empty()) return out;
    @autoreleasepool {
        cv::Mat src = roi.area() > 0 ? bgr(roi).clone() : bgr;
        CIImage* ci = matToCIImage(src);
        VNDetectRectanglesRequest* req = [[VNDetectRectanglesRequest alloc] init];
        req.maximumObservations = max_n;
        req.minimumAspectRatio  = min_aspect;
        req.minimumConfidence   = min_conf;
        req.minimumSize         = 0.02f;
        VNImageRequestHandler* h = [[VNImageRequestHandler alloc] initWithCIImage:ci options:@{}];
        NSError* err = nil; [h performRequests:@[req] error:&err];
        if (err) return out;
        for (VNRectangleObservation* o in req.results) {
            Quad q;
            q.confidence = o.confidence;
            q.corners[0] = vnToCv(o.topLeft,    src.cols, src.rows);
            q.corners[1] = vnToCv(o.topRight,   src.cols, src.rows);
            q.corners[2] = vnToCv(o.bottomRight,src.cols, src.rows);
            q.corners[3] = vnToCv(o.bottomLeft, src.cols, src.rows);
            if (roi.area() > 0)
                for (auto& c : q.corners) { c.x += roi.x; c.y += roi.y; }
            out.push_back(q);
        }
    }
    return out;
}

std::vector<std::vector<cv::Point2f>>
detectContours(const cv::Mat& bgr, float contrast, int max_dim) {
    std::vector<std::vector<cv::Point2f>> out;
    if (bgr.empty()) return out;
    @autoreleasepool {
        cv::Mat src = bgr;
        if (std::max(src.cols, src.rows) > max_dim) {
            const double k = (double)max_dim / std::max(src.cols, src.rows);
            cv::resize(bgr, src, cv::Size(), k, k, cv::INTER_AREA);
        }
        CIImage* ci = matToCIImage(src);
        VNDetectContoursRequest* req = [[VNDetectContoursRequest alloc] init];
        req.contrastAdjustment = contrast;
        req.detectsDarkOnLight = YES;
        VNImageRequestHandler* h = [[VNImageRequestHandler alloc] initWithCIImage:ci options:@{}];
        NSError* err = nil; [h performRequests:@[req] error:&err];
        if (err) return out;
        VNContoursObservation* obs = req.results.firstObject;
        if (!obs) return out;
        for (NSInteger i = 0; i < obs.contourCount; ++i) {
            VNContour* c = [obs contourAtIndex:i error:nil];
            if (!c) continue;
            std::vector<cv::Point2f> poly;
            const simd_float2* pts = (const simd_float2*)c.normalizedPoints;
            poly.reserve(c.pointCount);
            for (NSInteger k = 0; k < c.pointCount; ++k) {
                poly.emplace_back(pts[k].x * src.cols, (1.0f - pts[k].y) * src.rows);
            }
            out.push_back(std::move(poly));
        }
    }
    return out;
}

std::vector<float> featurePrint(const cv::Mat& bgr, const cv::Rect& roi) {
    std::vector<float> out;
    if (bgr.empty()) return out;
    @autoreleasepool {
        cv::Mat src = roi.area() > 0 ? bgr(roi).clone() : bgr;
        CIImage* ci = matToCIImage(src);
        VNGenerateImageFeaturePrintRequest* req = [[VNGenerateImageFeaturePrintRequest alloc] init];
        VNImageRequestHandler* h = [[VNImageRequestHandler alloc] initWithCIImage:ci options:@{}];
        NSError* err = nil; [h performRequests:@[req] error:&err];
        if (err) return out;
        VNFeaturePrintObservation* fp = req.results.firstObject;
        if (!fp) return out;
        out.resize(fp.elementCount);
        if (fp.elementType == VNElementTypeFloat) {
            memcpy(out.data(), fp.data.bytes, fp.elementCount * sizeof(float));
        } else {
            // Convert from doubles.
            const double* d = (const double*)fp.data.bytes;
            for (NSUInteger i = 0; i < fp.elementCount; ++i) out[i] = (float)d[i];
        }
    }
    return out;
}

float cosineDistance(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() != b.size() || a.empty()) return 2.0f;
    double dot = 0, na = 0, nb = 0;
    for (size_t i = 0; i < a.size(); ++i) { dot += a[i]*b[i]; na += a[i]*a[i]; nb += b[i]*b[i]; }
    if (na <= 0 || nb <= 0) return 2.0f;
    return (float)(1.0 - dot / std::sqrt(na * nb));
}

}  // namespace mate::vision
