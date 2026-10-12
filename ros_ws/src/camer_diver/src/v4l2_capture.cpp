#include "camer_diver/v4l2_capture.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace camer_diver
{
namespace
{
int xioctl(int fd, unsigned long request, void * arg)
{
  int result;
  do {result = ioctl(fd, request, arg);} while (result < 0 && errno == EINTR);
  return result;
}

void checked_ioctl(int fd, unsigned long request, void * arg, const char * name)
{
  if (xioctl(fd, request, arg) < 0) {
    throw std::runtime_error(std::string(name) + ": " + std::strerror(errno));
  }
}

uint32_t parse_format(const std::string & format)
{
  if (format == "MJPG") {return V4L2_PIX_FMT_MJPEG;}
  if (format == "YUYV") {return V4L2_PIX_FMT_YUYV;}
  if (format == "UYVY") {return V4L2_PIX_FMT_UYVY;}
  throw std::invalid_argument("pixel_format must be MJPG, YUYV or UYVY");
}
}  // namespace

V4L2Capture::~V4L2Capture() {close_device();}

void V4L2Capture::open_device(const std::string & device, uint32_t width,
  uint32_t height, uint32_t fps, const std::string & format)
{
  close_device();
  const auto requested_format = parse_format(format);
  if (width == 0 || height == 0 || width > 4096 || height > 4096 || fps == 0 || fps > 240) {
    throw std::invalid_argument("width/height must be 1..4096, fps must be 1..240");
  }
  struct stat status {};
  if (stat(device.c_str(), &status) < 0) {
    throw std::runtime_error("Cannot access " + device + ": " + std::strerror(errno));
  }
  if (!S_ISCHR(status.st_mode)) {
    throw std::runtime_error(device + " is not a character device");
  }
  fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("Cannot open " + device + ": " + std::strerror(errno));
  }

  try {
    v4l2_capability caps {};
    checked_ioctl(fd_, VIDIOC_QUERYCAP, &caps, "VIDIOC_QUERYCAP");
    const auto flags = (caps.capabilities & V4L2_CAP_DEVICE_CAPS) ?
      caps.device_caps : caps.capabilities;
    if (!(flags & V4L2_CAP_VIDEO_CAPTURE) || !(flags & V4L2_CAP_STREAMING)) {
      throw std::runtime_error("Device does not support single-plane V4L2 capture + streaming; "
              "select the capture node, not a metadata/codec device");
    }

    v4l2_format fmt {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = requested_format;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    checked_ioctl(fd_, VIDIOC_S_FMT, &fmt, "VIDIOC_S_FMT");
    if (fmt.fmt.pix.pixelformat != requested_format) {
      throw std::runtime_error("Camera rejected pixel_format=" + format +
              "; inspect v4l2-ctl --list-formats-ext and choose a supported format");
    }
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    format_ = fmt.fmt.pix.pixelformat;
    stride_ = fmt.fmt.pix.bytesperline;
    if (!width_ || !height_ || width_ > 4096 || height_ > 4096) {
      throw std::runtime_error("Camera returned invalid or excessive dimensions");
    }
    if (format_ != V4L2_PIX_FMT_MJPEG && (width_ % 2 || stride_ < width_ * 2)) {
      throw std::runtime_error("Invalid packed YUV width or bytesperline");
    }

    v4l2_streamparm parameters {};
    parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    checked_ioctl(fd_, VIDIOC_G_PARM, &parameters, "VIDIOC_G_PARM");
    if (parameters.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
      parameters.parm.capture.timeperframe.numerator = 1;
      parameters.parm.capture.timeperframe.denominator = fps;
      checked_ioctl(fd_, VIDIOC_S_PARM, &parameters, "VIDIOC_S_PARM");
    }
    const auto & interval = parameters.parm.capture.timeperframe;
    description_ = std::string(reinterpret_cast<const char *>(caps.card)) + " " +
      std::to_string(width_) + "x" + std::to_string(height_) + " " + format +
      " fps=" + (interval.numerator ?
      std::to_string(static_cast<double>(interval.denominator) / interval.numerator) :
      std::string("device default"));

    v4l2_requestbuffers request {};
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    request.count = 4;
    checked_ioctl(fd_, VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS");
    if (request.count < 2 || request.count > 64) {
      throw std::runtime_error("Camera returned an invalid mmap buffer count");
    }
    buffers_.reserve(request.count);
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer {};
      buffer.type = request.type;
      buffer.memory = request.memory;
      buffer.index = index;
      checked_ioctl(fd_, VIDIOC_QUERYBUF, &buffer, "VIDIOC_QUERYBUF");
      if (!buffer.length || buffer.length > 64U * 1024U * 1024U) {
        throw std::runtime_error("Invalid mmap buffer size");
      }
      void * address = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
        MAP_SHARED, fd_, buffer.m.offset);
      if (address == MAP_FAILED) {
        throw std::runtime_error(std::string("mmap: ") + std::strerror(errno));
      }
      buffers_.push_back({address, buffer.length});
      checked_ioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF");
    }
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    checked_ioctl(fd_, VIDIOC_STREAMON, &type, "VIDIOC_STREAMON");
    streaming_ = true;
  } catch (...) {
    close_device();
    throw;
  }
}

bool V4L2Capture::read_frame(Frame & frame)
{
  if (!streaming_) {throw std::runtime_error("Camera is not streaming");}
  pollfd descriptor {fd_, POLLIN, 0};
  const int ready = poll(&descriptor, 1, 0);
  if (ready < 0 && errno == EINTR) {return false;}
  if (ready < 0) {throw std::runtime_error(std::string("poll: ") + std::strerror(errno));}
  if (!ready) {return false;}
  if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error("Camera disconnected or reported a streaming error");
  }
  if (!(descriptor.revents & POLLIN)) {return false;}
  v4l2_buffer buffer {};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
    if (errno == EAGAIN) {return false;}
    throw std::runtime_error(std::string("VIDIOC_DQBUF: ") + std::strerror(errno));
  }
  try {
    if (buffer.index >= buffers_.size() || buffer.bytesused > buffers_[buffer.index].length) {
      throw std::runtime_error("V4L2 returned an invalid frame index or byte count");
    }
    if (!(buffer.flags & V4L2_BUF_FLAG_ERROR) && buffer.bytesused) {
      const auto * data = static_cast<const uint8_t *>(buffers_[buffer.index].address);
      frame.data.assign(data, data + buffer.bytesused);
      frame.width = width_;
      frame.height = height_;
      frame.stride = stride_;
      frame.pixel_format = format_;
    } else {
      frame.data.clear();
    }
  } catch (...) {
    // Return the buffer even when copying or validation fails.
    xioctl(fd_, VIDIOC_QBUF, &buffer);
    throw;
  }
  checked_ioctl(fd_, VIDIOC_QBUF, &buffer, "VIDIOC_QBUF");
  return !frame.data.empty();
}

void V4L2Capture::close_device() noexcept
{
  if (fd_ >= 0 && streaming_) {
    auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
  }
  streaming_ = false;
  for (const auto & buffer : buffers_) {munmap(buffer.address, buffer.length);}
  buffers_.clear();
  if (fd_ >= 0) {::close(fd_);}
  fd_ = -1;
}
}  // namespace camer_diver
