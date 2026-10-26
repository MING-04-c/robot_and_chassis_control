#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace camer_diver
{
struct Frame
{
  std::vector<uint8_t> data;
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0;
  uint32_t pixel_format = 0;
};

// Owns the device and all mmap buffers. Only access from one thread.
class V4L2Capture
{
public:
  V4L2Capture() = default;
  ~V4L2Capture();
  V4L2Capture(const V4L2Capture &) = delete;
  V4L2Capture & operator=(const V4L2Capture &) = delete;

  void open_device(const std::string & device, uint32_t width, uint32_t height,
    uint32_t fps, const std::string & format);
  // Returns false when no frame is ready or a driver marks a frame as corrupt.
  bool read_frame(Frame & frame);
  void close_device() noexcept;

  uint32_t width() const {return width_;}
  uint32_t height() const {return height_;}
  const std::string & description() const {return description_;}

private:
  struct Buffer {void * address; size_t length;};
  int fd_ = -1;
  bool streaming_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t stride_ = 0;
  uint32_t format_ = 0;
  std::string description_;
  std::vector<Buffer> buffers_;
};
}  // namespace camer_diver
