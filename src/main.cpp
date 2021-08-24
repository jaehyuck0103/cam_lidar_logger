#include <iostream>
#include <toml.hpp>

int main() {

    // Read config file
    const auto toml_data = toml::parse("./config.toml");
    const auto lidars_cfg = toml::find<std::vector<toml::table>>(toml_data, "lidars");

    for (const auto &lidar_cfg : lidars_cfg) {
        std::string lidar_model = toml::get<std::string>(lidar_cfg.at("lidar_model"));
        bool dual_return_mode = toml::get<bool>(lidar_cfg.at("dual_return_mode"));
        int fps = toml::get<int>(lidar_cfg.at("fps"));
        uint16_t start_azimuth = toml::get<uint16_t>(lidar_cfg.at("start_azimuth"));
        std::string lidar_ip = toml::get<std::string>(lidar_cfg.at("lidar_ip"));
        int recv_port = toml::get<int>(lidar_cfg.at("recv_port"));

        std::cout << lidar_model << std::endl;
        std::cout << dual_return_mode << std::endl;
        std::cout << fps << std::endl;
        std::cout << start_azimuth << std::endl;
        std::cout << lidar_ip << std::endl;
        std::cout << recv_port << std::endl;
    }
}
