#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: SSD1306 OLED panel backend that subscribes to DisplaySurface frames and flushes over I2C.
constructor_args:
  - config:
      expr: SSD1306::Config{"i2c_oled", 0x3C, 128, 64, "display_frame", 64}
template_args: []
required_hardware:
  - i2c_oled
depends:
  - DisplaySurface
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "DisplaySurface.hpp"
#include "app_framework.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "timebase.hpp"

class SSD1306 : public LibXR::Application
{
 public:
  struct Config
  {
    const char* i2c_alias;
    std::uint8_t address;
    std::uint16_t width;
    std::uint16_t height;
    const char* frame_topic_name;
    std::uint16_t chunk_bytes;
  };

  SSD1306(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          Config config)
      : address_(config.address),
        width_(CheckedWidth(config.width)),
        height_(CheckedHeight(config.height)),
        pages_(CalculatePages(height_)),
        framebuffer_size_(CalculateFramebufferSize(width_, pages_)),
        chunk_payload_size_(NormalizeChunkSize(config.chunk_bytes)),
        i2c_(hw.template FindOrExit<LibXR::I2C>({config.i2c_alias})),
        frame_topic_(LibXR::Topic::CreateTopic<DisplaySurface::Frame>(
            config.frame_topic_name)),
        frame_sub_(frame_topic_),
        last_frame_(new std::uint8_t[framebuffer_size_]{})
  {
    app.Register(*this);
    (void)i2c_->SetConfig({400000U});
    frame_sub_.StartWaiting();

    initialized_ = Init();
    if (initialized_)
    {
      XR_LOG_PASS("SSD1306: init succeeded");
    }
    else
    {
      XR_LOG_WARN("SSD1306: init failed, will retry");
    }
  }

  bool IsInitialized() const { return initialized_; }
  std::uint16_t Width() const { return width_; }
  std::uint16_t Height() const { return height_; }

  void OnMonitor() override
  {
    if (frame_sub_.Available())
    {
      pending_frame_ = frame_sub_.GetData();
      has_pending_frame_ = true;
      frame_sub_.StartWaiting();
    }

    if (!initialized_)
    {
      RetryInit();
      return;
    }

    if (!has_pending_frame_)
    {
      return;
    }

    const auto ans = FlushFrame(pending_frame_);
    if (ans == LibXR::ErrorCode::OK)
    {
      has_pending_frame_ = false;
    }
    else
    {
      const auto now_ms = static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds());
      if ((now_ms - last_error_log_ms_) > ERROR_LOG_INTERVAL_MS)
      {
        last_error_log_ms_ = now_ms;
        XR_LOG_WARN("SSD1306: flush failed %d", static_cast<int>(ans));
      }
    }
  }

 private:
  static constexpr std::uint8_t CTRL_COMMAND = 0x00;
  static constexpr std::uint8_t CTRL_DATA = 0x40;
  static constexpr std::uint8_t CMD_DISPLAY_OFF = 0xAE;
  static constexpr std::uint8_t CMD_DISPLAY_ON = 0xAF;
  static constexpr std::uint8_t CMD_SET_MEMORY_MODE = 0x20;
  static constexpr std::uint8_t CMD_SET_COLUMN_ADDRESS = 0x21;
  static constexpr std::uint8_t CMD_SET_PAGE_ADDRESS = 0x22;
  static constexpr std::uint8_t CMD_SET_START_LINE = 0x40;
  static constexpr std::uint8_t CMD_SET_CONTRAST = 0x81;
  static constexpr std::uint8_t CMD_SET_SEGMENT_REMAP = 0xA1;
  static constexpr std::uint8_t CMD_SET_NORMAL_DISPLAY = 0xA6;
  static constexpr std::uint8_t CMD_SET_MULTIPLEX = 0xA8;
  static constexpr std::uint8_t CMD_SET_COM_SCAN_DEC = 0xC8;
  static constexpr std::uint8_t CMD_SET_DISPLAY_OFFSET = 0xD3;
  static constexpr std::uint8_t CMD_SET_CLOCK_DIV = 0xD5;
  static constexpr std::uint8_t CMD_SET_PRECHARGE = 0xD9;
  static constexpr std::uint8_t CMD_SET_COM_PINS = 0xDA;
  static constexpr std::uint8_t CMD_SET_VCOM_DESELECT = 0xDB;
  static constexpr std::uint8_t CMD_SET_CHARGE_PUMP = 0x8D;
  static constexpr std::uint8_t CMD_ENTIRE_DISPLAY_RAM = 0xA4;

  static constexpr std::size_t COMMAND_PAYLOAD_CHUNK = 16;
  static constexpr std::size_t COMMAND_PACKET_SIZE = COMMAND_PAYLOAD_CHUNK + 1U;
  static constexpr std::size_t MAX_DATA_PAYLOAD = 64;
  static constexpr std::size_t DATA_PACKET_SIZE = MAX_DATA_PAYLOAD + 1U;
  static constexpr std::uint16_t DIRTY_SPLIT_GAP = 16U;
  static constexpr std::uint32_t INIT_RETRY_INTERVAL_MS = 500;
  static constexpr std::uint32_t ERROR_LOG_INTERVAL_MS = 1000;
  static constexpr std::uint16_t MAX_WIDTH = 128;
  static constexpr std::uint16_t MAX_HEIGHT = 64;

  struct FlushBounds
  {
    std::uint16_t x_begin = 0;
    std::uint16_t x_end = 0;
    std::uint16_t page_begin = 0;
    std::uint16_t page_end = 0;
  };

  static constexpr std::uint16_t NormalizeChunkSize(std::uint16_t chunk_bytes)
  {
    if (chunk_bytes < 16U)
    {
      return 16U;
    }
    if (chunk_bytes > MAX_DATA_PAYLOAD)
    {
      return static_cast<std::uint16_t>(MAX_DATA_PAYLOAD);
    }
    return chunk_bytes;
  }

  static std::uint16_t CheckedWidth(std::uint16_t width)
  {
    ASSERT(width != 0U && width <= MAX_WIDTH);
    return width;
  }

  static std::uint16_t CheckedHeight(std::uint16_t height)
  {
    ASSERT(height != 0U && height <= MAX_HEIGHT);
    return height;
  }

  static std::uint16_t CalculatePages(std::uint16_t height)
  {
    return static_cast<std::uint16_t>((height + 7U) >> 3U);
  }

  static std::size_t CalculateFramebufferSize(std::uint16_t width,
                                              std::uint16_t pages)
  {
    return static_cast<std::size_t>(width) * pages;
  }

  std::uint8_t GetComPinsConfig() const { return height_ <= 32U ? 0x02 : 0x12; }

  bool Init()
  {
    last_frame_valid_ = false;

    const std::uint8_t init_commands[] = {
        CMD_DISPLAY_OFF,
        CMD_SET_CLOCK_DIV,
        0x80,
        CMD_SET_MULTIPLEX,
        static_cast<std::uint8_t>(height_ - 1U),
        CMD_SET_DISPLAY_OFFSET,
        0x00,
        CMD_SET_START_LINE,
        CMD_SET_SEGMENT_REMAP,
        CMD_SET_COM_SCAN_DEC,
        CMD_SET_COM_PINS,
        GetComPinsConfig(),
        CMD_SET_CONTRAST,
        0x7F,
        CMD_ENTIRE_DISPLAY_RAM,
        CMD_SET_NORMAL_DISPLAY,
        CMD_SET_PRECHARGE,
        0xF1,
        CMD_SET_VCOM_DESELECT,
        0x40,
        CMD_SET_CHARGE_PUMP,
        0x14,
        CMD_SET_MEMORY_MODE,
        0x00,
    };
    static constexpr std::uint8_t DISPLAY_ON[] = {CMD_DISPLAY_ON};

    if (WriteCommands(init_commands, sizeof(init_commands)) != LibXR::ErrorCode::OK)
    {
      return false;
    }

    if (Clear() != LibXR::ErrorCode::OK)
    {
      return false;
    }

    LibXR::Thread::Sleep(20);
    return WriteCommands(DISPLAY_ON, sizeof(DISPLAY_ON)) == LibXR::ErrorCode::OK;
  }

  void RetryInit()
  {
    const auto now_ms = static_cast<std::uint32_t>(LibXR::Timebase::GetMilliseconds());
    if ((now_ms - last_init_retry_ms_) < INIT_RETRY_INTERVAL_MS)
    {
      return;
    }

    last_init_retry_ms_ = now_ms;
    initialized_ = Init();
    if (initialized_)
    {
      XR_LOG_PASS("SSD1306: init recovered");
    }
  }

  LibXR::ErrorCode FlushFrame(const DisplaySurface::Frame& frame)
  {
    if (frame.data == nullptr)
    {
      return LibXR::ErrorCode::PTR_NULL;
    }
    if (frame.pixel_format != DisplaySurface::PixelFormat::MONO_VTILED_LSB ||
        frame.width != width_ || frame.height != height_ || frame.pitch != width_ ||
        frame.size < static_cast<std::uint32_t>(framebuffer_size_))
    {
      return LibXR::ErrorCode::ARG_ERR;
    }

    FlushBounds bounds{};
    if (!ResolveFlushBounds(frame, bounds))
    {
      return LibXR::ErrorCode::OK;
    }

    if (!last_frame_valid_)
    {
      bounds = {0, width_, 0, pages_};
      const auto ans = FlushPageRanges(frame, bounds);
      if (ans == LibXR::ErrorCode::OK)
      {
        StoreFrameBounds(frame, bounds);
        last_frame_valid_ = true;
      }
      return ans;
    }

    const auto ans = FlushDirtyRanges(frame, bounds);
    if (ans == LibXR::ErrorCode::OK)
    {
      StoreFrameBounds(frame, bounds);
    }
    return ans;
  }

  LibXR::ErrorCode Clear()
  {
    auto ans = SetFullWindow();
    if (ans != LibXR::ErrorCode::OK)
    {
      return ans;
    }

    std::fill(data_packet_.begin(), data_packet_.end(), 0);
    data_packet_[0] = CTRL_DATA;

    std::size_t remaining = framebuffer_size_;
    while (remaining > 0U)
    {
      const std::size_t payload = std::min<std::size_t>(chunk_payload_size_, remaining);
      ans = i2c_->Write(address_, LibXR::ConstRawData(data_packet_.data(), payload + 1U),
                        write_op_);
      if (ans != LibXR::ErrorCode::OK)
      {
        return ans;
      }
      remaining -= payload;
    }

    std::fill(last_frame_, last_frame_ + framebuffer_size_, 0);
    last_frame_valid_ = true;
    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode SetFullWindow()
  {
    const std::uint8_t commands[] = {
        CMD_SET_MEMORY_MODE,
        0x00,
        CMD_SET_COLUMN_ADDRESS,
        0x00,
        static_cast<std::uint8_t>(width_ - 1U),
        CMD_SET_PAGE_ADDRESS,
        0x00,
        static_cast<std::uint8_t>(pages_ - 1U),
    };

    return WriteCommands(commands, sizeof(commands));
  }

  LibXR::ErrorCode SetWindow(std::uint16_t x, std::uint16_t page, std::uint16_t width,
                             std::uint16_t pages)
  {
    if (width == 0U || pages == 0U || x >= width_ || page >= pages_ ||
        (x + width) > width_ || (page + pages) > pages_)
    {
      return LibXR::ErrorCode::ARG_ERR;
    }

    const std::uint8_t commands[] = {
        CMD_SET_MEMORY_MODE,
        0x00,
        CMD_SET_COLUMN_ADDRESS,
        static_cast<std::uint8_t>(x),
        static_cast<std::uint8_t>(x + width - 1U),
        CMD_SET_PAGE_ADDRESS,
        static_cast<std::uint8_t>(page),
        static_cast<std::uint8_t>(page + pages - 1U),
    };

    return WriteCommands(commands, sizeof(commands));
  }

  static std::uint16_t ClipEnd(std::uint16_t begin, std::uint16_t length,
                               std::uint16_t limit)
  {
    if (begin >= limit)
    {
      return limit;
    }

    const std::uint32_t end =
        static_cast<std::uint32_t>(begin) + static_cast<std::uint32_t>(length);
    return end > limit ? limit : static_cast<std::uint16_t>(end);
  }

  bool ResolveFlushBounds(const DisplaySurface::Frame& frame, FlushBounds& bounds) const
  {
    if (frame.full_update)
    {
      bounds = {0, width_, 0, pages_};
      return true;
    }

    if (frame.dirty_width == 0U || frame.dirty_height == 0U ||
        frame.x >= width_ || frame.y >= height_)
    {
      return false;
    }

    const std::uint16_t x_end = ClipEnd(frame.x, frame.dirty_width, width_);
    const std::uint16_t y_end = ClipEnd(frame.y, frame.dirty_height, height_);
    const std::uint16_t page_begin = static_cast<std::uint16_t>(frame.y >> 3U);
    const std::uint16_t page_end = static_cast<std::uint16_t>((y_end + 7U) >> 3U);

    if (x_end <= frame.x || page_end <= page_begin)
    {
      return false;
    }

    bounds = {frame.x, x_end, page_begin, std::min<std::uint16_t>(page_end, pages_)};
    return bounds.page_end > bounds.page_begin;
  }

  LibXR::ErrorCode FlushPageRanges(const DisplaySurface::Frame& frame,
                                   const FlushBounds& bounds)
  {
    for (std::uint16_t page = bounds.page_begin; page < bounds.page_end; ++page)
    {
      const std::uint16_t width = bounds.x_end - bounds.x_begin;
      auto ans = SetWindow(bounds.x_begin, page, width, 1U);
      if (ans != LibXR::ErrorCode::OK)
      {
        return ans;
      }

      const std::size_t offset =
          static_cast<std::size_t>(page) * frame.pitch + bounds.x_begin;
      ans = WriteData(frame.data + offset, width);
      if (ans != LibXR::ErrorCode::OK)
      {
        return ans;
      }
    }

    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode FlushDirtyRanges(const DisplaySurface::Frame& frame,
                                    const FlushBounds& bounds)
  {
    for (std::uint16_t page = bounds.page_begin; page < bounds.page_end; ++page)
    {
      const std::size_t last_page_offset = static_cast<std::size_t>(page) * width_;
      const std::size_t frame_page_offset = static_cast<std::size_t>(page) * frame.pitch;
      std::uint16_t pos = bounds.x_begin;

      while (pos < bounds.x_end)
      {
        while (pos < bounds.x_end &&
               last_frame_[last_page_offset + pos] ==
                   frame.data[frame_page_offset + pos])
        {
          ++pos;
        }
        if (pos >= bounds.x_end)
        {
          break;
        }

        const std::uint16_t run_start = pos;
        std::uint16_t last_dirty = pos;
        std::uint16_t clean_count = 0;
        ++pos;

        while (pos < bounds.x_end)
        {
          if (last_frame_[last_page_offset + pos] != frame.data[frame_page_offset + pos])
          {
            last_dirty = pos;
            clean_count = 0;
          }
          else
          {
            ++clean_count;
            if (clean_count >= DIRTY_SPLIT_GAP)
            {
              pos = static_cast<std::uint16_t>(last_dirty + 1U);
              break;
            }
          }
          ++pos;
        }

        const std::uint16_t run_width =
            static_cast<std::uint16_t>(last_dirty - run_start + 1U);
        auto ans = SetWindow(run_start, page, run_width, 1U);
        if (ans != LibXR::ErrorCode::OK)
        {
          return ans;
        }

        ans = WriteData(frame.data + frame_page_offset + run_start, run_width);
        if (ans != LibXR::ErrorCode::OK)
        {
          return ans;
        }
      }
    }

    return LibXR::ErrorCode::OK;
  }

  void StoreFrameBounds(const DisplaySurface::Frame& frame, const FlushBounds& bounds)
  {
    const std::size_t width = bounds.x_end - bounds.x_begin;
    for (std::uint16_t page = bounds.page_begin; page < bounds.page_end; ++page)
    {
      const std::size_t last_offset =
          static_cast<std::size_t>(page) * width_ + bounds.x_begin;
      const std::size_t frame_offset =
          static_cast<std::size_t>(page) * frame.pitch + bounds.x_begin;
      std::memcpy(last_frame_ + last_offset, frame.data + frame_offset, width);
    }
  }

  LibXR::ErrorCode WriteCommands(const std::uint8_t* commands, std::size_t size)
  {
    if (commands == nullptr && size != 0U)
    {
      return LibXR::ErrorCode::PTR_NULL;
    }

    std::size_t offset = 0;
    while (offset < size)
    {
      const std::size_t payload =
          std::min<std::size_t>(COMMAND_PAYLOAD_CHUNK, size - offset);
      command_packet_[0] = CTRL_COMMAND;
      std::memcpy(command_packet_.data() + 1U, commands + offset, payload);

      auto ans = i2c_->Write(
          address_, LibXR::ConstRawData(command_packet_.data(), payload + 1U), write_op_);
      if (ans != LibXR::ErrorCode::OK)
      {
        return ans;
      }

      offset += payload;
    }

    return LibXR::ErrorCode::OK;
  }

  LibXR::ErrorCode WriteData(const std::uint8_t* data, std::size_t size)
  {
    if (data == nullptr && size != 0U)
    {
      return LibXR::ErrorCode::PTR_NULL;
    }

    std::size_t offset = 0;
    while (offset < size)
    {
      const std::size_t payload =
          std::min<std::size_t>(chunk_payload_size_, size - offset);
      data_packet_[0] = CTRL_DATA;
      std::memcpy(data_packet_.data() + 1U, data + offset, payload);

      auto ans = i2c_->Write(
          address_, LibXR::ConstRawData(data_packet_.data(), payload + 1U), write_op_);
      if (ans != LibXR::ErrorCode::OK)
      {
        return ans;
      }

      offset += payload;
    }

    return LibXR::ErrorCode::OK;
  }

  std::uint8_t address_ = 0;
  std::uint16_t width_ = 0;
  std::uint16_t height_ = 0;
  std::uint16_t pages_ = 0;
  std::size_t framebuffer_size_ = 0;
  std::uint16_t chunk_payload_size_ = 0;
  LibXR::I2C* i2c_ = nullptr;
  LibXR::Topic frame_topic_;
  LibXR::Topic::ASyncSubscriber<DisplaySurface::Frame> frame_sub_;
  LibXR::WriteOperation write_op_;
  DisplaySurface::Frame pending_frame_{};
  std::array<std::uint8_t, COMMAND_PACKET_SIZE> command_packet_{};
  std::array<std::uint8_t, DATA_PACKET_SIZE> data_packet_{};
  std::uint8_t* last_frame_ = nullptr;
  std::uint32_t last_init_retry_ms_ = 0;
  std::uint32_t last_error_log_ms_ = 0;
  bool initialized_ = false;
  bool has_pending_frame_ = false;
  bool last_frame_valid_ = false;
};
