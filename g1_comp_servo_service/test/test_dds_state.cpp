#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "param.h"

int main(int argc, char** argv) {
    param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(param::domain_id, param::network_interface);

    std::atomic<int> received{0};
    auto subscriber = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(
        "rt/g1_comp_servo/state");
    subscriber->InitChannel(
        [&received](const void* message) {
            const auto& state = *static_cast<const unitree_go::msg::dds_::MotorStates_*>(message);
            if (state.states().size() < 2) {
                std::cout << "Received state with " << state.states().size() << " motors." << std::endl;
                return;
            }
            ++received;
            std::cout << "state q(deg): yaw=" << state.states()[0].q()
                      << " pitch=" << state.states()[1].q() << std::endl;
        },
        1);

    std::cout << "Listening to rt/g1_comp_servo/state for 5 seconds (read-only)." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (received == 0) {
        std::cerr << "No G1-Comp state received." << std::endl;
        return 1;
    }
    std::cout << "Received " << received.load() << " state messages." << std::endl;
    return 0;
}
