/*
 * Author: Andrew Campbell
 * Date: 06-09-2024
 */

// Config
#include <../Config/Config.hpp>

// STL
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

int main()
{
    const std::string configPath = "Config.yml";
    const Algorithms::Configuration::Config config = Algorithms::Configuration::loadConfig(configPath);

    std::cout << "config Loaded" << std::endl;

    return 0;
}

