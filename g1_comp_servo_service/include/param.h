#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline std::string device;
inline std::string ns;
inline uint16_t damping;
inline std::string arm_direction;
inline std::string network_interface;
inline std::string config_file;
inline int domain_id = 0;


/* ---------- 命令行参数 ---------- */
void helper(int argc, char** argv)
{
#ifndef NDEBUG
    spdlog::set_level(spdlog::level::debug);
#else
    spdlog::set_level(spdlog::level::info);
#endif

    device = "/dev/ttyUSB0";
    if (const char* ros_domain_id = std::getenv("ROS_DOMAIN_ID")) {
        domain_id = std::stoi(ros_domain_id);
    }
    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--help" || option == "-h") {
            std::cout
                << "Options:\n"
                << "  -s, --serial <port>    serial port (default /dev/ttyUSB0)\n"
                << "  -n, --network <iface> DDS network interface\n"
                << "      --domain <id>     DDS domain (default ROS_DOMAIN_ID or 0)\n"
                << "  -c, --config <path>    calibration config.yaml\n";
            exit(0);
        }
        if (i + 1 >= argc) {
            spdlog::error("Missing value for {}", option);
            exit(1);
        }
        const std::string value = argv[++i];
        if (option == "--serial" || option == "-s") device = value;
        else if (option == "--network" || option == "-n") network_interface = value;
        else if (option == "--domain") domain_id = std::stoi(value);
        else if (option == "--config" || option == "-c") config_file = value;
        else {
            spdlog::error("Unknown option: {}", option);
            exit(1);
        }
    }
}

inline std::string resolve_config_path(const char* argv0)
{
    if (!config_file.empty()) {
        return std::filesystem::absolute(config_file).string();
    }

    const std::filesystem::path exe_dir =
        std::filesystem::absolute(argv0).parent_path();
    const std::vector<std::filesystem::path> candidates = {
        exe_dir / "config.yaml",
        exe_dir.parent_path() / "config" / "config.yaml",
        std::filesystem::current_path() / "config" / "config.yaml",
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate.string();
        }
    }
    spdlog::error("Cannot find config.yaml; pass --config <path>");
    exit(1);
}

/* ---------- config.yaml ---------- */
inline struct {
    float dt;
    float pos_limit;

    std::vector<float> kp;
    std::vector<float> kd;

    struct {
        std::string network;
    } dds;
} cfg;
inline YAML::Node config;
void parse_config()
{
    // 获取执行程序的路径
    auto get_exe_path = []() -> std::string {
        std::vector<char> path(1024);
        ssize_t len = readlink("/proc/self/exe", &path[0], path.size());
        if (len != -1) {
            path[len] = '\0';  // Null-terminate the result
            return std::string(&path[0]);
        } else {
            spdlog::error("Failed to get executable path.");
            exit(1);
        }
    };

    std::filesystem::path exe_path = std::filesystem::path(get_exe_path());
    std::string config_path = exe_path.parent_path().string() + "/config.yaml";

    // 如果config.yaml不存在(并非部署时)，则是正在开发阶段，使用默认路径
    if (!std::filesystem::exists(config_path)) {
        config_path = exe_path.parent_path().parent_path().string() + "/config/config.yaml";
    }
    spdlog::debug("Config Path: {}", config_path);

    try {
        config = YAML::LoadFile(config_path);

        cfg.dt = config["dt"].as<float>();
        cfg.pos_limit = config["vel_limit"].as<float>() * cfg.dt; // 直接在此处转为单次角度限幅
        cfg.dds.network = (config["dds"]["network"]) ? config["dds"]["network"].as<std::string>() : "";
        cfg.kp = config["kp"].as<std::vector<float>>();
        cfg.kd = config["kd"].as<std::vector<float>>();

    } catch (const std::exception& e) {
        spdlog::error("Failed to parse config.yaml: {}", e.what());
        exit(1);
    }
}

}

#endif // PARAM_H
