#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include "dxl.h"

namespace {
std::atomic<bool> keep_running{true};

void stop_reader(int) {
    keep_running = false;
}
}  // namespace

int main(int argc, char** argv) {
    param::helper(argc, argv);
    dxl::Motors<2> joints(1000000);

    // This is a passive read test. Disable torque before reading either motor.
    for (int id = 0; id < 2; ++id) {
        joints.enable(id, 0);
    }

    std::signal(SIGINT, stop_reader);
    std::signal(SIGTERM, stop_reader);
    std::cout << "Torque disabled. Reading encoder feedback; press Ctrl+C to exit."
              << std::endl;

    while (keep_running) {
        joints.sync_get_position();
        std::cout << "servo 0 position: " << joints.present_position[0] << '\n'
                  << "servo 1 position: " << joints.present_position[1] << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    for (int id = 0; id < 2; ++id) {
        joints.enable(id, 0);
    }
    std::cout << "Both servos released." << std::endl;
    return 0;
}
