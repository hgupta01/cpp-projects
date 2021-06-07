#include <iostream>
#include <yaml-cpp/yaml.h>


int main() {
  YAML::Node config = YAML::LoadFile(
      "/home/himanshu/Downloads/cpp_projects/yaml_parser/config.yaml");

  const std::string filename = config["filename"].as<std::string>();
std::vector<double> array = config["array"].as<std::vector<double>>();
  return 0;
}
