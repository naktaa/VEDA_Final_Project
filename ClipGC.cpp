#include "ClipGC.hpp"
#include "Utils.hpp"
#include <filesystem>
#include <chrono>
#include <thread>

namespace fs = std::filesystem;

ClipGC::ClipGC(std::string clip_root_dir, int ttl_sec, int interval_ms)
  : clip_root_dir_(std::move(clip_root_dir)), ttl_sec_(ttl_sec), interval_ms_(interval_ms) {}

ClipGC::~ClipGC(){ stop(); }

void ClipGC::start(){
  if (running_) return;
  running_ = true;
  th_ = std::thread(&ClipGC::loop, this);
}

void ClipGC::stop(){
  running_ = false;
  if (th_.joinable()) th_.join();
}

void ClipGC::loop(){
  while (running_) {
    std::error_code ec;
    if (fs::exists(clip_root_dir_, ec)) {
      auto now = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

      for (auto& p : fs::recursive_directory_iterator(clip_root_dir_, ec)) {
        if (ec) break;
        if (!p.is_regular_file(ec)) continue;
        if (p.path().extension() != ".mp4") continue;

        auto mt = utils::file_mtime_epoch_sec(p.path().string());
        if (!mt.has_value()) continue;
        if (now - *mt >= ttl_sec_) utils::remove_file(p.path().string());
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
  }
}
