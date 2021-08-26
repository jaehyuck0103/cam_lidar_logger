#include "pandar/pandar_config.h"
#include "pandar/pandar_lidar_receiver.h"
#include "pandar/pandar_utils.h"

#include <fmt/chrono.h>
#include <fmt/core.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <tbb/concurrent_queue.h>
#include <toml.hpp>

#include <chrono>
#include <iostream>
#include <memory>

#ifdef OLD_COMPILER
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#endif

namespace chrono = std::chrono;
using std::chrono::system_clock;
using LidarQueue =
    tbb::concurrent_bounded_queue<std::tuple<int, std::vector<uint8_t>, time_point<system_clock>>>;
using namespace std::chrono_literals;

std::string getFormattedTime(const time_point<system_clock> &tp) {

    std::string date = fmt::format("{:%y%m%d_%H%M%S}", tp);

    const auto tse = tp.time_since_epoch();
    const auto ms = chrono::duration_cast<chrono::milliseconds>(tse) -
                    chrono::duration_cast<chrono::seconds>(tse);
    const auto us = chrono::duration_cast<chrono::microseconds>(tse) -
                    chrono::duration_cast<chrono::milliseconds>(tse);

    return fmt::format("{:%y%m%d_%H%M%S}_{:03d}_{:03d}", tp, ms.count(), us.count());
}

void captureCamera(int cam_id, const fs::path &save_root) {

#if 1
    std::string video_path = "videos/" + std::to_string(cam_id) + ".mkv";
    cv::VideoCapture cap{video_path};
#else
    cv::VideoCapture cap{cam_id};
#endif

    if (cap.isOpened()) {
        std::cout << fmt::format("Cam {:02d} open\n", cam_id);
    } else {
        std::cout << fmt::format("Cam {:02d} fail to open\n", cam_id);
        return;
    }

    const fs::path cam_save_dir = save_root / fmt::format("cam{:02d}", cam_id);
    fs::create_directories(cam_save_dir);

    cv::Mat frame;
    while (cap.read(frame)) {

        const fs::path jpg_path =
            cam_save_dir / fmt::format("{}.jpg", getFormattedTime(system_clock::now()));

        cv::imwrite(jpg_path, frame, {cv::IMWRITE_JPEG_QUALITY, 100});
    }

    std::cout << fmt::format("Cam {:02d} end\n", cam_id);
}

fs::path parseSaveRoot(std::string str_root) {

    // Expand Home Directory
    const std::string home_dir = std::getenv("HOME");
    if (str_root.rfind("~", 0) == 0) {
        str_root = home_dir + str_root.substr(1);
    }

    // Append subdirectory of current system time
    const fs::path root =
        fs::path{str_root} / fmt::format("{:%y%m%d_%H%M%S}", system_clock::now());

    // Create directories
    fs::create_directories(root);

    return root;
}

int main() {

    std::cout << "\n\n\n\n\n";

    // Read config file
    const auto toml_data = toml::parse("./config.toml");

    // save root
    fs::path save_root = parseSaveRoot(toml::find<std::string>(toml_data, "save_root"));

    // Init lidar receviers
    LidarQueue lidar_queue;
    lidar_queue.set_capacity(5);

    std::vector<PandarConfig> pandar_cfgs;
    std::vector<std::unique_ptr<PandarLidarReceiver>> pandar_lidar_receivers;

    const auto lidars_cfg = toml::find<std::vector<toml::table>>(toml_data, "lidars");
    for (const auto &lidar_cfg : lidars_cfg) {
        std::string lidar_model_str = toml::get<std::string>(lidar_cfg.at("lidar_model"));
        bool dual_return_mode = toml::get<bool>(lidar_cfg.at("dual_return_mode"));
        int fps = toml::get<int>(lidar_cfg.at("fps"));
        uint16_t start_azimuth = toml::get<uint16_t>(lidar_cfg.at("start_azimuth"));
        std::string lidar_ip = toml::get<std::string>(lidar_cfg.at("lidar_ip"));
        uint16_t recv_port = toml::get<uint16_t>(lidar_cfg.at("recv_port"));

        auto angle_corrections = PandarUtils::getFallbackAngleCorrection("Pandar64");
        if (!angle_corrections.has_value()) {
            std::cout << "Fail to get Angle Correction\n";
            std::abort();
        }

        pandar_cfgs.emplace_back(
            lidar_model_from_str[lidar_model_str],
            dual_return_mode,
            fps,
            start_azimuth,
            angle_corrections->first,
            angle_corrections->second);

        pandar_lidar_receivers
            .emplace_back(std::make_unique<PandarLidarReceiver>(
                lidar_ip,
                recv_port,
                [&lidar_queue, lidar_id{pandar_cfgs.size() - 1}](const auto &a, const auto &b) {
                    lidar_queue.try_push({lidar_id, a, b});
                },
                pandar_cfgs.back()))
            ->start();
    }

    // Save PandarConfigs
    for (size_t lidar_id = 0; lidar_id < pandar_cfgs.size(); ++lidar_id) {
        const fs::path json_save_path = save_root / fmt::format("Lidar{:02d}.json", lidar_id);
        PandarUtils::writePandarConfigJson(json_save_path, pandar_cfgs[lidar_id]);

        const fs::path lidar_save_dir = save_root / fmt::format("Lidar{:02d}", lidar_id);
        fs::create_directories(lidar_save_dir);
    }

    // Launch camera recv threads
    std::vector<std::thread> camera_threads;
    for (int cam_id = 0; cam_id < 8; ++cam_id) {
        camera_threads.emplace_back([=]() { captureCamera(cam_id, save_root); });
    }

    // Lidar Receive Handling
    std::tuple<int, std::vector<uint8_t>, time_point<system_clock>> ret;
    while (true) {
        lidar_queue.pop(ret);
        const auto &[lidar_id, frame_buffer, timestamp] = ret;

        const fs::path lidar_save_path = save_root / fmt::format("Lidar{:02d}", lidar_id) /
                                         fmt::format("{}.fb", getFormattedTime(timestamp));

        auto fp = std::fstream(lidar_save_path, std::ios::out | std::ios::binary);
        fp.write((char *)&frame_buffer[0], frame_buffer.size());
        fp.close();
    }
}
