#include <iostream>
#include <string>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

namespace {

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program << " --network <iface> [--domain <id>] [--release-motion]\n"
        << "Without --release-motion this only reports the active official motion mode.\n"
        << "With --release-motion it asks the official motion switcher to release control.\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string network;
    int domain = 0;
    bool release_motion = false;

    for (int i = 1; i < argc; ++i) {
        const std::string option = argv[i];
        if (option == "--help" || option == "-h") {
            print_usage(argv[0]);
            return 0;
        }
        if (option == "--release-motion") {
            release_motion = true;
            continue;
        }
        if ((option == "--network" || option == "-n") && i + 1 < argc) {
            network = argv[++i];
            continue;
        }
        if (option == "--domain" && i + 1 < argc) {
            domain = std::stoi(argv[++i]);
            continue;
        }
        std::cerr << "Unknown or incomplete option: " << option << '\n';
        print_usage(argv[0]);
        return 2;
    }
    if (network.empty()) {
        std::cerr << "--network is required.\n";
        return 2;
    }

    unitree::robot::ChannelFactory::Instance()->Init(domain, network);
    unitree::robot::b2::MotionSwitcherClient switcher;
    switcher.SetTimeout(5.0F);
    switcher.Init();

    std::string form;
    std::string name;
    int32_t result = switcher.CheckMode(form, name);
    if (result != 0) {
        std::cerr << "CheckMode failed with error code " << result << ".\n";
        return 1;
    }
    if (name.empty()) {
        std::cout << "Official motion control is already released.\n";
        return 0;
    }
    std::cout << "Official motion control is active: form='" << form
              << "', mode='" << name << "'.\n";
    if (!release_motion) {
        std::cout << "No change was made. Re-run with --release-motion only when the robot is safely supported.\n";
        return 0;
    }

    result = switcher.ReleaseMode();
    if (result != 0) {
        std::cerr << "ReleaseMode failed with error code " << result << ".\n";
        return 1;
    }
    std::cout << "ReleaseMode succeeded.\n";
    return 0;
}
