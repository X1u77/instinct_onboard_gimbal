#include "dxl.h"
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    param::helper(argc, argv);
    auto joints = dxl::Motors<2>(1000000);

    for(int id=0;id<2;id++)
    {
        joints.enable(id, 0);
    }

    while(true)
    {   
        std::cout<<std::endl;
        joints.sync_get_position();
        std::cout<<"servo 0 position : "<<joints.present_position[0]<<std::endl;
        std::cout<<"servo 1 position : "<<joints.present_position[1]<<std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}
