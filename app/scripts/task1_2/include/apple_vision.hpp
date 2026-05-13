#pragma once
// ─────────────────────────────────────────────────────────────────────────
// apple_vision.hpp — Apple Vision framework integration.
//
// Apple Silicon / macOS only. Vision is Apple's high-level computer-vision
// framework backed by the Neural Engine. We use it for:
//   • VNDetectRectanglesRequest — finds quadrilateral candidates with
//     sub-pixel corners in a single call. Used to refine plate corners
//     after the HSV pre-pass.
//   • VNDetectContoursRequest — finds dark/light contour boundaries.
//     Used as a complementary signal for pipe edges (combined with our
//     Metal Sobel/Canny output).
//   • VNGenerateImageFeaturePrintRequest — for cross-pair plate
//     identification when colors degrade in low light.
//
// When Vision isn't available (non-Apple build) all functions return
// empty results and the caller falls back to OpenCV-only paths.
// ─────────────────────────────────────────────────────────────────────────

#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace mate::vision {

struct Quad {
    std::array<cv::Point2f, 4> corners{};   // TL, TR, BR, BL (image coords)
    float confidence = 0.0f;                // 0..1
};

// Detect quadrilateral candidates within an image region. `roi` may be
// empty (full image). `min_aspect_ratio` filters obviously-non-square
// candidates. Returns up to `max_n` candidates sorted by descending
// confidence.
std::vector<Quad>
detectRectangles(const cv::Mat& bgr,
                 const cv::Rect& roi = {},
                 int max_n = 16,
                 float min_aspect_ratio = 0.5f,
                 float min_confidence = 0.3f);

// Detect dark contours (closed boundaries). Returns each contour as a
// polyline in image coordinates. Used to seed pipe-edge detection.
std::vector<std::vector<cv::Point2f>>
detectContours(const cv::Mat& bgr,
               float contrast_adjustment = 1.0f,
               int max_image_dimension = 1024);

// Compute a Vision feature print (a 2048-dim L2-normalized embedding) for
// the given image patch. Returns the embedding as a vector of floats, or
// empty on failure. Useful for matching plates across viewpoints when
// color matching is ambiguous.
std::vector<float>
featurePrint(const cv::Mat& bgr, const cv::Rect& roi = {});

// Cosine distance between two feature prints (range [0..2]; 0 = identical).
float cosineDistance(const std::vector<float>& a, const std::vector<float>& b);

}  // namespace mate::vision
