#pragma once
#include <atomic>
#include <string>
#include <thread>

class ClipGC {
public:
  ClipGC(std::string clip_root_dir, int ttl_sec, int interval_ms);
  ~ClipGC();

  void start();
  void stop();

private:
  void loop();

  std::string clip_root_dir_;
  int ttl_sec_;
  int interval_ms_;
  std::atomic<bool> running_{false};
  std::thread th_;
};
