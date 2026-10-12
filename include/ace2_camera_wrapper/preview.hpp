// Small 8-bit BGR preview of a published frame, for the operator video stream.
//
// The full frame is 5 MP, 10 MB per frame at 12 bits, and too large to send to
// another process at the camera rate. The preview is the frame shrunk by a whole
// factor with a box average: for a Bayer frame each output pixel averages every
// red, green and blue sample in its block, which is a demosaic and a downscale
// in one pass with no interpolation. 16-bit samples are scaled from the
// camera's significant bits down to 8, so a 12-bit frame is not dark.
//
// Header only and free of pylon and OpenCV, so it can be tested on its own.
#ifndef ACE2_CAMERA_WRAPPER__PREVIEW_HPP_
#define ACE2_CAMERA_WRAPPER__PREVIEW_HPP_

#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace ace2_camera_wrapper
{

// Where red and blue sit in a 2x2 Bayer cell, as (row, column), for the four
// ROS Bayer encodings. The other two positions are green.
struct BayerPhase
{
  int r_row, r_col, b_row, b_col;
};

inline bool bayerPhase(const std::string & encoding, BayerPhase & phase, int & bytes)
{
  if (encoding.rfind("bayer_", 0) != 0 || encoding.size() < 11) {return false;}
  const std::string base = encoding.substr(0, 10);
  const std::string bits = encoding.substr(10);
  if (bits == "8") {bytes = 1;} else if (bits == "16") {bytes = 2;} else {return false;}
  if (base == "bayer_rggb") {phase = {0, 0, 1, 1};}
  else if (base == "bayer_bggr") {phase = {1, 1, 0, 0};}
  else if (base == "bayer_gbrg") {phase = {1, 0, 0, 1};}
  else if (base == "bayer_grbg") {phase = {0, 1, 1, 0};}
  else {return false;}
  return true;
}

// Significant bits in a 16-bit sample, from the camera's PixelFormat name:
// BayerRG12 and Mono12 carry 12, BayerRG10 10. 16 when the name has no number.
inline int significantBits(const std::string & pixel_format)
{
  size_t i = pixel_format.size();
  while (i > 0 && pixel_format[i - 1] >= '0' && pixel_format[i - 1] <= '9') {--i;}
  if (i == pixel_format.size()) {return 16;}
  const int bits = std::stoi(pixel_format.substr(i));
  return (bits >= 8 && bits <= 16) ? bits : 16;
}

// One raw sample, 8 or 16 bits wide (little endian, as the camera delivers it).
inline uint32_t sampleAt(const uint8_t * row, uint32_t x, int bytes)
{
  if (bytes == 1) {return row[x];}
  uint16_t v;
  std::memcpy(&v, row + 2 * static_cast<size_t>(x), 2);
  return v;
}

// Fills out with a bgr8 preview of in, factor times smaller on each side (an even
// factor for Bayer frames). Returns false, leaving out untouched, for an encoding
// it cannot read or a factor that does not fit the frame.
inline bool makePreview(
  const sensor_msgs::msg::Image & in, int factor, int bits16, sensor_msgs::msg::Image & out)
{
  if (factor < 1 || in.width == 0 || in.height == 0 || in.data.empty()) {return false;}
  BayerPhase phase{};
  int bytes = 1;
  const bool bayer = bayerPhase(in.encoding, phase, bytes);
  int channels = 0;
  bool rgb_order = false;
  if (!bayer) {
    if (in.encoding == "bgr8") {channels = 3;}
    else if (in.encoding == "rgb8") {channels = 3; rgb_order = true;}
    else if (in.encoding == "mono8") {channels = 1;}
    else if (in.encoding == "mono16") {channels = 1; bytes = 2;}
    else {return false;}
  }
  if (bayer && factor % 2 != 0) {return false;}
  const uint32_t ow = in.width / static_cast<uint32_t>(factor);
  const uint32_t oh = in.height / static_cast<uint32_t>(factor);
  if (ow == 0 || oh == 0) {return false;}
  const size_t row_bytes = static_cast<size_t>(in.width) * bytes * (bayer ? 1 : channels);
  if (in.step < row_bytes || in.data.size() < static_cast<size_t>(in.step) * in.height) {return false;}
  const int shift = bytes == 2 ? std::clamp(bits16, 8, 16) - 8 : 0;

  out.header = in.header;
  out.width = ow;
  out.height = oh;
  out.encoding = "bgr8";
  out.is_bigendian = 0;
  out.step = ow * 3;
  out.data.assign(static_cast<size_t>(out.step) * oh, 0);

  std::vector<uint32_t> acc(static_cast<size_t>(ow) * 3);
  std::vector<uint32_t> cnt(static_cast<size_t>(ow) * 3);
  if (bayer) {
    // Whole 2x2 cells: each gives one red, two green and one blue sample, at
    // positions fixed by the phase, so the inner loop has no branches.
    int chan[2][2];   // BGR output channel of each cell position: 0 blue, 1 green, 2 red
    for (int r = 0; r < 2; ++r) {
      for (int c = 0; c < 2; ++c) {
        chan[r][c] = (r == phase.r_row && c == phase.r_col) ? 2
          : (r == phase.b_row && c == phase.b_col) ? 0 : 1;
      }
    }
    const uint32_t cells = static_cast<uint32_t>(factor / 2);
    const uint32_t per_block = cells * cells;   // red and blue samples per output pixel
    const uint32_t div[3] = {per_block, 2 * per_block, per_block};
    for (uint32_t oy = 0; oy < oh; ++oy) {
      std::fill(acc.begin(), acc.end(), 0u);
      for (uint32_t cy = 0; cy < cells; ++cy) {
        const size_t y = static_cast<size_t>(oy) * factor + 2 * cy;
        const uint8_t * r0 = in.data.data() + y * in.step;
        const uint8_t * r1 = r0 + in.step;
        for (uint32_t ox = 0; ox < ow; ++ox) {
          uint32_t * a = &acc[static_cast<size_t>(ox) * 3];
          for (uint32_t cx = 0; cx < cells; ++cx) {
            const uint32_t x = ox * static_cast<uint32_t>(factor) + 2 * cx;
            a[chan[0][0]] += sampleAt(r0, x, bytes);
            a[chan[0][1]] += sampleAt(r0, x + 1, bytes);
            a[chan[1][0]] += sampleAt(r1, x, bytes);
            a[chan[1][1]] += sampleAt(r1, x + 1, bytes);
          }
        }
      }
      uint8_t * dst = out.data.data() + static_cast<size_t>(oy) * out.step;
      for (size_t i = 0; i < acc.size(); ++i) {
        const uint32_t d = div[i % 3];
        dst[i] = static_cast<uint8_t>(std::min<uint32_t>(((acc[i] + d / 2) / d) >> shift, 255u));
      }
    }
    return true;
  }
  for (uint32_t oy = 0; oy < oh; ++oy) {
    std::fill(acc.begin(), acc.end(), 0u);
    std::fill(cnt.begin(), cnt.end(), 0u);
    for (int dy = 0; dy < factor; ++dy) {
      const uint32_t y = oy * static_cast<uint32_t>(factor) + static_cast<uint32_t>(dy);
      const uint8_t * row = in.data.data() + static_cast<size_t>(y) * in.step;
      for (uint32_t ox = 0; ox < ow; ++ox) {
        uint32_t * a = &acc[static_cast<size_t>(ox) * 3];
        uint32_t * n = &cnt[static_cast<size_t>(ox) * 3];
        for (int dx = 0; dx < factor; ++dx) {
          const uint32_t x = ox * static_cast<uint32_t>(factor) + static_cast<uint32_t>(dx);
          if (channels == 3) {
            const uint8_t * p = row + static_cast<size_t>(x) * 3;
            a[0] += rgb_order ? p[2] : p[0];
            a[1] += p[1];
            a[2] += rgb_order ? p[0] : p[2];
            n[0] += 1; n[1] += 1; n[2] += 1;
          } else {
            const uint32_t v = sampleAt(row, x, bytes);
            a[0] += v; a[1] += v; a[2] += v;
            n[0] += 1; n[1] += 1; n[2] += 1;
          }
        }
      }
    }
    uint8_t * dst = out.data.data() + static_cast<size_t>(oy) * out.step;
    for (size_t i = 0; i < acc.size(); ++i) {
      const uint32_t mean = cnt[i] ? (acc[i] + cnt[i] / 2) / cnt[i] : 0u;
      dst[i] = static_cast<uint8_t>(std::min<uint32_t>(mean >> shift, 255u));
    }
  }
  return true;
}

}  // namespace ace2_camera_wrapper

#endif  // ACE2_CAMERA_WRAPPER__PREVIEW_HPP_
