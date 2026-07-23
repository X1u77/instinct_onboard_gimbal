#include <iostream>
#include "yaml_parser.h"

int main()
{
    YamlParser yamlParam_;
    std::string env_cfg_path = param::resolve_config_path(argv[0]);
    yamlParam_.setup(env_cfg_path.c_str());

    float  dt = yamlParam_.ReadFloatFromYaml("dt");
    std::cout<<"dt: "<<dt<<std::endl;
}
