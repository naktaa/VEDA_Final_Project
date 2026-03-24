#include "auto_app.hpp"

#include <atomic>
#include <csignal>
#include <exception>
#include <filesystem>
#include <iostream>

#include "rc_config.hpp"
#include "rc_control_node.hpp"

namespace {

std::atomic<bool> g_run{true};

void OnSignal(int) {
    g_run = false;
}

} // namespace

void PrintAutoUsage(const char* exe) {
    std::cout << "사용법: " << exe << "\n";
    std::cout << "빌드 후 build 디렉터리에서 sudo ./main 으로 실행합니다.\n";
}

bool ParseAutoConfig(int argc, char** argv, RcAppConfig& config, std::string& error) {
    if (argc > 1) {
        error = "CLI 인자는 사용하지 않습니다. config/rc_control.ini를 수정해서 실행하세요.";
        return false;
    }
    return true;
}

void ResolveAutoPaths(const char* exe, RcAppConfig& config) {
    namespace fs = std::filesystem;

    fs::path exe_path = fs::absolute(exe);
    if (!exe_path.has_parent_path()) {
        exe_path = fs::current_path() / exe_path;
    }
    const fs::path build_dir = exe_path.parent_path();
    const fs::path repo_dir = build_dir.parent_path();

    config.config_dir = repo_dir / "config";
    config.ini_path = config.config_dir / "rc_control.ini";
    config.template_ini_path = config.config_dir / "rc_control.template.ini";
}

int RunAutoApp(const RcAppConfig& config) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    RcAppConfig runtime_config = config;
    bool created = false;
    std::string ensure_error;
    if (!EnsureRcLocalConfigExists(runtime_config.template_ini_path,
                                   runtime_config.ini_path,
                                   &created,
                                   &ensure_error)) {
        std::cerr << "[ERR] rc_control.ini 준비 실패: " << ensure_error << "\n";
        return 1;
    }

    if (created) {
        std::cout << "[INFO] 로컬 설정 파일 생성: " << runtime_config.ini_path << "\n";
        std::cout << "[INFO] 값을 수정한 뒤 다시 sudo ./main 으로 실행하세요.\n";
        return 0;
    }

    LoadRcParamsFromIni(runtime_config.ini_path.string(), runtime_config);

    RcControlNode node(runtime_config);
    if (!node.start()) {
        return 1;
    }

    node.run(&g_run);
    node.stop();
    return 0;
}
