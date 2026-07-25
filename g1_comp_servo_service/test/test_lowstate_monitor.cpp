#include <chrono>
#include <iostream>
#include <thread>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "param.h"

int main(int argc, char** argv) {
    param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(param::domain_id, param::network_interface);

    constexpr size_t kLeftElbowMotor = 18;  // sim joint 9 -> real motor 18
    auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(1);
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
        "rt/lowstate");
    subscriber->InitChannel([&](const void* message) {
        const auto& state = *static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
        if (state.motor_state().size() <= kLeftElbowMotor) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - last_print < std::chrono::milliseconds(250)) {
            return;
        }
        last_print = now;
        const auto& motor = state.motor_state()[kLeftElbowMotor];
        std::cout << "left_elbow real[18]: q=" << motor.q()
                  << " dq=" << motor.dq() << " tau_est=" << motor.tau_est()
                  << " | mode_machine=" << static_cast<unsigned>(state.mode_machine())
                  << std::endl;
    }, 1);

    std::cout << "Read-only monitor of rt/lowstate for 10 seconds." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    return 0;
}
