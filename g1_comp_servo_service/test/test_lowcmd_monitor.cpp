#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "param.h"

int main(int argc, char** argv) {
    param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(param::domain_id, param::network_interface);

    constexpr size_t kLeftElbowMotor = 18;  // sim joint 9 -> real motor 18
    auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>>(
        "rt/lowcmd");
    subscriber->InitChannel([&](const void* message) {
        const auto& command = *static_cast<const unitree_hg::msg::dds_::LowCmd_*>(message);
        if (command.motor_cmd().size() <= kLeftElbowMotor) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_print < std::chrono::milliseconds(250)) {
            return;
        }
        last_print = now;
        const auto& motor = command.motor_cmd()[kLeftElbowMotor];
        std::cout << "left_elbow real[18]: mode=" << static_cast<unsigned>(motor.mode())
                  << " q=" << motor.q() << " dq=" << motor.dq()
                  << " kp=" << motor.kp() << " kd=" << motor.kd()
                  << " | mode_pr=" << static_cast<unsigned>(command.mode_pr())
                  << " mode_machine=" << static_cast<unsigned>(command.mode_machine())
                  << std::endl;
    }, 1);

    std::cout << "Read-only monitor of rt/lowcmd for 10 seconds. "
              << "Start ColdStart in another terminal while this runs." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
}
