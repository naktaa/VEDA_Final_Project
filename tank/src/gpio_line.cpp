#include "gpio_line.hpp"

#include <cerrno>
#include <cstring>
#include <string>

#include <gpiod.h>

#include "timebase.hpp"

namespace {

std::string errno_string(const std::string& context) {
    return context + ": " + std::strerror(errno);
}

void set_error(std::string* error, const std::string& message) {
    if (error) {
        *error = message;
    }
}

} // namespace

GpioEdgeLine::~GpioEdgeLine() {
    close();
}

bool GpioEdgeLine::open_rising_edge(const std::string& chip_path,
                                    unsigned int offset,
                                    const std::string& consumer,
                                    std::string* error) {
    close();

    chip_path_ = chip_path;
    offset_ = offset;
    chip_ = gpiod_chip_open(chip_path.c_str());
    if (!chip_) {
        set_error(error, errno_string("failed to open GPIO chip " + chip_path));
        return false;
    }

#ifdef USE_LIBGPIOD_V2
    gpiod_line_settings* settings = gpiod_line_settings_new();
    gpiod_line_config* line_config = gpiod_line_config_new();
    gpiod_request_config* request_config = gpiod_request_config_new();
    if (!settings || !line_config || !request_config) {
        set_error(error, "failed to allocate libgpiod edge request state");
        if (request_config) gpiod_request_config_free(request_config);
        if (line_config) gpiod_line_config_free(line_config);
        if (settings) gpiod_line_settings_free(settings);
        close();
        return false;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_DOWN);
    gpiod_request_config_set_consumer(request_config, consumer.c_str());
    if (gpiod_line_config_add_line_settings(line_config, &offset_, 1, settings) < 0) {
        set_error(error, errno_string("failed to configure GPIO edge line"));
        gpiod_request_config_free(request_config);
        gpiod_line_config_free(line_config);
        gpiod_line_settings_free(settings);
        close();
        return false;
    }

    request_ = gpiod_chip_request_lines(chip_, request_config, line_config);
    gpiod_request_config_free(request_config);
    gpiod_line_config_free(line_config);
    gpiod_line_settings_free(settings);
    if (!request_) {
        set_error(error, errno_string("failed to request GPIO edge line"));
        close();
        return false;
    }

    event_buffer_ = gpiod_edge_event_buffer_new(8);
    if (!event_buffer_) {
        set_error(error, "failed to allocate GPIO edge event buffer");
        close();
        return false;
    }
#else
    line_ = gpiod_chip_get_line(chip_, offset_);
    if (!line_) {
        set_error(error, errno_string("failed to get GPIO line"));
        close();
        return false;
    }

#ifdef GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN
    if (gpiod_line_request_rising_edge_events_flags(
            line_, consumer.c_str(), GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN) < 0) {
        if (gpiod_line_request_rising_edge_events(line_, consumer.c_str()) < 0) {
            set_error(error, errno_string("failed to request GPIO edge line"));
            close();
            return false;
        }
    }
#else
    if (gpiod_line_request_rising_edge_events(line_, consumer.c_str()) < 0) {
        set_error(error, errno_string("failed to request GPIO edge line"));
        close();
        return false;
    }
#endif
#endif

    return true;
}

bool GpioEdgeLine::wait_for_rising_edge(int timeout_ms,
                                        int64_t* event_time_ns,
                                        std::string* error) {
    if (!is_open()) {
        set_error(error, "GPIO edge line is not open");
        return false;
    }

#ifdef USE_LIBGPIOD_V2
    const long long timeout_ns = static_cast<long long>(timeout_ms) * 1000000LL;
    const int wait_result = gpiod_line_request_wait_edge_events(request_, timeout_ns);
    if (wait_result < 0) {
        set_error(error, errno_string("GPIO edge wait failed"));
        return false;
    }
    if (wait_result == 0) {
        return false;
    }

    const int read_count = gpiod_line_request_read_edge_events(request_, event_buffer_, 8);
    if (read_count < 0) {
        set_error(error, errno_string("GPIO edge read failed"));
        return false;
    }
#else
    timespec timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_nsec = static_cast<long>(timeout_ms % 1000) * 1000000L;
    const int wait_result = gpiod_line_event_wait(line_, &timeout);
    if (wait_result < 0) {
        set_error(error, errno_string("GPIO edge wait failed"));
        return false;
    }
    if (wait_result == 0) {
        return false;
    }

    gpiod_line_event event {};
    if (gpiod_line_event_read(line_, &event) < 0) {
        set_error(error, errno_string("GPIO edge read failed"));
        return false;
    }
#endif

    if (event_time_ns) {
        *event_time_ns = clock_ns(CLOCK_MONOTONIC_RAW);
    }
    return true;
}

void GpioEdgeLine::close() {
#ifdef USE_LIBGPIOD_V2
    if (event_buffer_) {
        gpiod_edge_event_buffer_free(event_buffer_);
        event_buffer_ = nullptr;
    }
    if (request_) {
        gpiod_line_request_release(request_);
        request_ = nullptr;
    }
#else
    if (line_) {
        gpiod_line_release(line_);
        line_ = nullptr;
    }
#endif
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
    chip_path_.clear();
    offset_ = 0;
}

bool GpioEdgeLine::is_open() const {
#ifdef USE_LIBGPIOD_V2
    return request_ != nullptr;
#else
    return line_ != nullptr;
#endif
}
