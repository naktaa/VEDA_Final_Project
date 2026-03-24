#pragma once

#include <cstdint>
#include <string>

struct gpiod_chip;
struct gpiod_line;
struct gpiod_line_request;
struct gpiod_edge_event_buffer;

class GpioOutputLine {
public:
    GpioOutputLine() = default;
    ~GpioOutputLine();

    GpioOutputLine(const GpioOutputLine&) = delete;
    GpioOutputLine& operator=(const GpioOutputLine&) = delete;

    bool open(const std::string& chip_path,
              unsigned int offset,
              const std::string& consumer,
              int initial_value,
              std::string* error = nullptr);
    bool set_value(int value, std::string* error = nullptr);
    void close();

    bool is_open() const;

private:
    std::string chip_path_;
    unsigned int offset_ = 0;
    gpiod_chip* chip_ = nullptr;
#ifdef USE_LIBGPIOD_V2
    gpiod_line_request* request_ = nullptr;
#else
    gpiod_line* line_ = nullptr;
#endif
};

class GpioEdgeLine {
public:
    GpioEdgeLine() = default;
    ~GpioEdgeLine();

    GpioEdgeLine(const GpioEdgeLine&) = delete;
    GpioEdgeLine& operator=(const GpioEdgeLine&) = delete;

    bool open_rising_edge(const std::string& chip_path,
                          unsigned int offset,
                          const std::string& consumer,
                          std::string* error = nullptr);
    bool wait_for_rising_edge(int timeout_ms,
                              int64_t* event_time_ns,
                              std::string* error = nullptr);
    void close();

    bool is_open() const;

private:
    std::string chip_path_;
    unsigned int offset_ = 0;
    gpiod_chip* chip_ = nullptr;
#ifdef USE_LIBGPIOD_V2
    gpiod_line_request* request_ = nullptr;
    gpiod_edge_event_buffer* event_buffer_ = nullptr;
#else
    gpiod_line* line_ = nullptr;
#endif
};
