#include "dxl.h"
#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "utilities.h"
#include "yaml_parser.h"


class IOUSB
{
public:
    IOUSB(const char* argv0)
    : joints(1000000)
    {
        motorstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>(
            "rt/g1_comp_servo/state");
        motorstate->msg_.states().resize(2);
        motorcmd = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/g1_comp_servo/cmd");
        {
            std::lock_guard<std::mutex> lock(motorcmd->mutex_);
            motorcmd->msg_.cmds().resize(2);
        }


        for (int i(0); i<2; i++)
        {
            joints.enable(i, 0);
            joints.set_position_p_gain(i, 500);
            joints.set_position_d_gain(i, 300);
        }
        spdlog::info("Set gain success.");

        std::string env_cfg_path = param::resolve_config_path(argv0);
        yamlParam_.setup(env_cfg_path.c_str());

        float has_calibrate = yamlParam_.ReadFloatFromYaml("has_calibrate");
        if(!has_calibrate) 
        {
            std::cout<<std::endl;
            std::cout<<" please run test_calibration first! "<<std::endl;
            exit(-1);
        }

        servo0_calibration = yamlParam_.ReadFloatFromYaml("servo0_calibration");
        servo1_calibration = yamlParam_.ReadFloatFromYaml("servo1_calibration");

        joint0_limitation.setZero(2);
        joint1_limitation.setZero(2);
        
        joint0_limitation = yamlParam_.ReadVectorFromYaml("joint0",2);
        joint1_limitation = yamlParam_.ReadVectorFromYaml("joint1",2);

        servo0_limit_enoder = yamlParam_.ReadFloatFromYaml("servo0_calibration");
        servo1_limit_enoder = yamlParam_.ReadFloatFromYaml("servo1_calibration");

        direction.setZero(2);
        direction = yamlParam_.ReadVectorFromYaml("direction",2);

        servo_angle.setZero(2);

        target_angle_command.setZero(2);
    }

    void Start()
    {
        thread_ = std::make_shared<unitree::common::RecurrentThread>(
           0.015 * 1000, std::bind(&IOUSB::run, this));
    }
    dxl::Motors<2> joints;

private:
    void run()
    {
        joints.sync_get_position();
        servo_angle(0) = utilities::encoder2angle(joints.present_position[0],servo0_limit_enoder,joint0_limitation,direction(0));
        servo_angle(1) = utilities::encoder2angle(joints.present_position[1],servo1_limit_enoder,joint1_limitation,direction(1));
        motorstate->lock();
        for(int i(0); i<2; i++)
        {
            motorstate->msg_.states()[i].q(servo_angle(i));
        }
        motorstate->unlockAndPublish();

        if (motorcmd->isTimeout())
        {
            set_motor_enable(false);
            return;
        }

        unitree_go::msg::dds_::MotorCmds_ command;
        {
            std::lock_guard<std::mutex> lock(motorcmd->mutex_);
            command = motorcmd->msg_;
        }
        if (command.cmds().size() < MOTOR_NUM)
        {
            spdlog::warn("G1-Comp command contains fewer than two motors.");
            set_motor_enable(false);
            return;
        }
        const bool should_enable = command.cmds()[0].mode() == 1;
        if (!should_enable)
        {
            set_motor_enable(false);
            return;
        }

        for(int i(0); i<2; i++)
        {   
            target_angle_command(i) = command.cmds()[i].q();
            float servo_encoder_positon;
            if(i == 0)
                servo_encoder_positon = utilities::angle2encoder(target_angle_command(i),servo0_limit_enoder,joint0_limitation,direction(0));
            if(i == 1)
                servo_encoder_positon = utilities::angle2encoder(target_angle_command(i),servo1_limit_enoder,joint1_limitation,direction(1));
            joints.set_position(i, servo_encoder_positon);
        }
        // Write the first target while torque is still off, then enable.
        // This avoids moving toward a stale Goal Position at startup.
        set_motor_enable(true);
    }

    void set_motor_enable(bool enable)
    {
        if(enable)
        {
            if(!joint_enable)
            {
                for(int i(0); i<MOTOR_NUM; i++)
                {
                    joints.enable(i);
                }
                joint_enable = true;
                spdlog::info("Enable arm.");
            }
        }
        else
        {
            if(joint_enable)
            {
                for(int i(0); i<MOTOR_NUM; i++)
                {
                    joints.enable(i, 0);
                }
                joint_enable = false;
                spdlog::info("Release arm.");
            }
        }
    }

    unitree::common::RecurrentThreadPtr thread_;
    bool joint_enable = false;

    /* DDS */
    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> motorstate; // motor state
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>> motorcmd; // motor cmd
    YamlParser yamlParam_;
    float servo0_calibration;
    float servo1_calibration;
    Eigen::VectorXf joint0_limitation;
    Eigen::VectorXf joint1_limitation;
    float servo0_limit_enoder;
    float servo1_limit_enoder;
    Eigen::VectorXf direction;
    Eigen::VectorXf servo_angle;

    Eigen::VectorXf target_angle_command;
    Eigen::VectorXf target_encoder_command;
};

int main(int argc, char** argv)
{
    param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(param::domain_id, param::network_interface);
    auto iousb = IOUSB(argv[0]);
    iousb.Start();
    while(true)
    {
        sleep(1);
    }
    return 0;
}
