#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include "../include/clearpath.h"

using json = nlohmann::json;

static std::atomic<bool> keep_running{true};

static std::mutex husky_mutex;
static std::mutex target_mutex;

static int server_fd_global = -1;

static double max_speed = 0.5;
static double max_accel = 0.5;

static double target_left = 0.0;
static double target_right = 0.0;
static double target_left_accel = 0.5;
static double target_right_accel = 0.5;

static std::chrono::steady_clock::time_point last_cmd_time =
    std::chrono::steady_clock::now();

static constexpr double COMMAND_TIMEOUT_SEC = 0.1;

static json ok()
{
    return {{"ok", true}};
}

static json error(const std::string& msg)
{
    return {{"ok", false}, {"error", msg}};
}

static void sigint_handler(int)
{
    keep_running = false;

    if (server_fd_global >= 0) {
        close(server_fd_global);
        server_fd_global = -1;
    }
}

static void husky_command_loop()
{
    while (keep_running) {
        try {
            double left;
            double right;
            double left_accel;
            double right_accel;

            {
                std::lock_guard<std::mutex> lock(target_mutex);

                auto now = std::chrono::steady_clock::now();
                double dt = std::chrono::duration<double>(now - last_cmd_time).count();

                if (dt > COMMAND_TIMEOUT_SEC) {
                    left = 0.0;
                    right = 0.0;
                    left_accel = max_accel;
                    right_accel = max_accel;
                } else {
                    left = target_left;
                    right = target_right;
                    left_accel = target_left_accel;
                    right_accel = target_right_accel;
                }
            }

            {
                std::lock_guard<std::mutex> lock(husky_mutex);
                clearpath::SetDifferentialSpeed(left, right, left_accel, right_accel).send();
            }
        }
        catch (clearpath::Exception* ex) {
            std::cerr << "Clearpath exception in command loop: " << ex->message << "\n";
        }
        catch (const std::exception& ex) {
            std::cerr << "std::exception in command loop: " << ex.what() << "\n";
        }
        catch (...) {
            std::cerr << "Unknown exception in command loop\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

static json get_system_status()
{
    auto* system = clearpath::DataSystemStatus::getUpdate(0.20);

    if (!system) {
        return error("No system status data returned");
    }

    json voltages = json::array();
    json currents = json::array();
    json temperatures = json::array();

    for (unsigned int i = 0; i < system->getVoltagesCount(); ++i) {
        voltages.push_back(system->getVoltage(i));
    }

    for (unsigned int i = 0; i < system->getCurrentsCount(); ++i) {
        currents.push_back(system->getCurrent(i));
    }

    for (unsigned int i = 0; i < system->getTemperaturesCount(); ++i) {
        temperatures.push_back(system->getTemperature(i));
    }

    json out = {
        {"ok", true},
        {"uptime_ms", system->getUptime()},
        {"voltages", voltages},
        {"currents", currents},
        {"temperatures", temperatures}
    };

    delete system;
    return out;
}

static json get_safety_status()
{
    auto* safety = clearpath::DataSafetySystemStatus::getUpdate(0.20);

    if (!safety) {
        return error("No safety status data returned");
    }

    uint16_t flags = safety->getFlags();

    json out = {
        {"ok", true},
        {"flags", flags},
        {"timeout", static_cast<bool>(flags & 0x1)},
        {"lockout", static_cast<bool>(flags & 0x2)},
        {"estop", static_cast<bool>(flags & 0x8)},
        {"cci", static_cast<bool>(flags & 0x10)},
        {"psu", static_cast<bool>(flags & 0x20)},
        {"current", static_cast<bool>(flags & 0x40)}
    };

    delete safety;
    return out;
}

static json get_encoders()
{
    auto* enc = clearpath::DataEncoders::getUpdate(0.20);

    if (!enc) {
        return error("No encoder data returned");
    }

    json encoders = json::array();

    for (unsigned int i = 0; i < enc->getCount(); ++i) {
        encoders.push_back({
            {"travel", enc->getTravel(i)},
            {"speed", enc->getSpeed(i)}
        });
    }

    json out = {
        {"ok", true},
        {"count", enc->getCount()},
        {"encoders", encoders}
    };

    delete enc;
    return out;
}

static json get_differential_speed()
{
    auto* speed = clearpath::DataDifferentialSpeed::getUpdate(0.20);

    if (!speed) {
        return error("No differential speed data returned");
    }

    json out = {
        {"ok", true},
        {"left_speed", speed->getLeftSpeed()},
        {"right_speed", speed->getRightSpeed()},
        {"left_accel", speed->getLeftAccel()},
        {"right_accel", speed->getRightAccel()}
    };

    delete speed;
    return out;
}

static json get_orientation()
{
    auto* orient = clearpath::DataPlatformOrientation::getUpdate(0.20);

    if (!orient) {
        return error("No orientation data returned");
    }

    json out = {
        {"ok", true},
        {"roll", orient->getRoll()},
        {"pitch", orient->getPitch()},
        {"yaw", orient->getYaw()}
    };

    delete orient;
    return out;
}

static json get_rotation()
{
    auto* rot = clearpath::DataPlatformRotation::getUpdate(0.20);

    if (!rot) {
        return error("No rotation data returned");
    }

    json out = {
        {"ok", true},
        {"roll_rate", rot->getRollRate()},
        {"pitch_rate", rot->getPitchRate()},
        {"yaw_rate", rot->getYawRate()}
    };

    delete rot;
    return out;
}

static json get_acceleration()
{
    auto* accel = clearpath::DataPlatformAcceleration::getUpdate(0.20);

    if (!accel) {
        return error("No acceleration data returned");
    }

    json out = {
        {"ok", true},
        {"x", accel->getX()},
        {"y", accel->getY()},
        {"z", accel->getZ()}
    };

    delete accel;
    return out;
}

static json get_platform_info()
{
    auto* info = clearpath::DataPlatformInfo::getUpdate(0.20);

    if (!info) {
        return error("No platform info returned");
    }

    json out = {
        {"ok", true},
        {"model", info->getModel()},
        {"revision", info->getRevision()},
        {"serial", info->getSerial()}
    };

    delete info;
    return out;
}

static json get_platform_name()
{
    auto* name = clearpath::DataPlatformName::getUpdate(0.20);

    if (!name) {
        return error("No platform name returned");
    }

    json out = {
        {"ok", true},
        {"name", name->getName()}
    };

    delete name;
    return out;
}

static json get_firmware_info()
{
    auto* fw = clearpath::DataFirmwareInfo::getUpdate(0.20);

    if (!fw) {
        return error("No firmware info returned");
    }

    auto t = fw->getWriteTime();

    json out = {
        {"ok", true},
        {"major_firmware", fw->getMajorFirmwareVersion()},
        {"minor_firmware", fw->getMinorFirmwareVersion()},
        {"major_protocol", fw->getMajorProtocolVersion()},
        {"minor_protocol", fw->getMinorProtocolVersion()},
        {"write_time", {
            {"minute", t.minute()},
            {"hour", t.hour()},
            {"day", t.day()},
            {"month", t.month()},
            {"year", t.year()}
        }}
    };

    delete fw;
    return out;
}

static json handle_request(const json& req)
{
    const std::string cmd = req.value("cmd", "");

    if (cmd == "ping") {
        return ok();
    }

    if (cmd == "stop") {
        std::lock_guard<std::mutex> lock(target_mutex);

        target_left = 0.0;
        target_right = 0.0;
        target_left_accel = max_accel;
        target_right_accel = max_accel;
        last_cmd_time = std::chrono::steady_clock::now();

        return ok();
    }

    if (cmd == "set_differential_speed") {
        std::lock_guard<std::mutex> lock(target_mutex);

        target_left = req.at("left_speed").get<double>();
        target_right = req.at("right_speed").get<double>();
        target_left_accel = req.value("left_accel", max_accel);
        target_right_accel = req.value("right_accel", max_accel);
        last_cmd_time = std::chrono::steady_clock::now();

        return ok();
    }

    if (cmd == "set_max_speed") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        double fwd = req.at("max_fwd").get<double>();
        double rev = req.at("max_rev").get<double>();

        max_speed = fwd;
        clearpath::SetMaxSpeed(fwd, rev).send();

        return ok();
    }

    if (cmd == "set_max_accel") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        double fwd = req.at("max_fwd").get<double>();
        double rev = req.at("max_rev").get<double>();

        max_accel = fwd;
        clearpath::SetMaxAccel(fwd, rev).send();

        return ok();
    }

    if (cmd == "set_differential_output") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        clearpath::SetDifferentialOutput(
            req.at("left").get<double>(),
            req.at("right").get<double>()
        ).send();

        return ok();
    }

    if (cmd == "set_velocity") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        clearpath::SetVelocity(
            req.at("translational").get<double>(),
            req.at("rotational").get<double>(),
            req.value("trans_accel", max_accel)
        ).send();

        return ok();
    }

    if (cmd == "set_turn") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        clearpath::SetTurn(
            req.at("translational").get<double>(),
            req.at("turn_radius").get<double>(),
            req.value("trans_accel", max_accel)
        ).send();

        return ok();
    }

    if (cmd == "set_ackermann_output") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        clearpath::SetAckermannOutput(
            req.at("steering").get<double>(),
            req.at("throttle").get<double>(),
            req.at("brake").get<double>()
        ).send();

        return ok();
    }

    if (cmd == "set_gear") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        clearpath::SetGear(req.at("gear").get<uint8_t>()).send();

        return ok();
    }

    if (cmd == "set_platform_name") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        std::string name = req.at("name").get<std::string>();
        clearpath::SetPlatformName(name.c_str()).send();

        return ok();
    }

    if (cmd == "set_platform_time") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        clearpath::SetPlatformTime(req.at("time").get<uint32_t>()).send();

        return ok();
    }

    if (cmd == "set_safety_system") {
        std::lock_guard<std::mutex> lock(husky_mutex);

        clearpath::SetSafetySystem(req.at("flags").get<uint16_t>()).send();

        return ok();
    }

    if (cmd == "store_settings") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        clearpath::CmdStoreSettings().send();
        return ok();
    }

    if (cmd == "restore_user_settings") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        clearpath::CmdRestoreSettings(
            clearpath::CmdRestoreSettings::USER_SETTINGS
        ).send();
        return ok();
    }

    if (cmd == "restore_factory_settings") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        clearpath::CmdRestoreSettings(
            clearpath::CmdRestoreSettings::FACTORY_SETTINGS
        ).send();
        return ok();
    }

    if (cmd == "processor_reset") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        clearpath::CmdProcessorReset().send();
        return ok();
    }

    if (cmd == "get_system_status") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_system_status();
    }

    if (cmd == "get_safety_status") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_safety_status();
    }

    if (cmd == "get_encoders") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_encoders();
    }

    if (cmd == "get_differential_speed") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_differential_speed();
    }

    if (cmd == "get_orientation") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_orientation();
    }

    if (cmd == "get_rotation") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_rotation();
    }

    if (cmd == "get_acceleration") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_acceleration();
    }

    if (cmd == "get_platform_info") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_platform_info();
    }

    if (cmd == "get_platform_name") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_platform_name();
    }

    if (cmd == "get_firmware_info") {
        std::lock_guard<std::mutex> lock(husky_mutex);
        return get_firmware_info();
    }

    return error("Unknown command: " + cmd);
}

static std::string read_line(int fd)
{
    std::string line;
    char c = 0;

    while (true) {
        ssize_t n = recv(fd, &c, 1, 0);

        if (n <= 0) {
            return "";
        }

        if (c == '\n') {
            break;
        }

        line.push_back(c);
    }

    return line;
}

static void client_loop(int client_fd)
{
    while (keep_running) {
        std::string line = read_line(client_fd);

        if (line.empty()) {
            break;
        }

        json response;

        try {
            json request = json::parse(line);
            response = handle_request(request);
        }
        catch (clearpath::Exception* ex) {
            response = error(ex->message);
        }
        catch (const std::exception& ex) {
            response = error(ex.what());
        }
        catch (...) {
            response = error("Unknown server error");
        }

        std::string out = response.dump() + "\n";
        send(client_fd, out.c_str(), out.size(), 0);
    }

    close(client_fd);
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, sigint_handler);
    std::signal(SIGTERM, sigint_handler);

    std::string serial_port = "/dev/husky-base";
    int socket_port = 9000;

    if (argc >= 2) {
        serial_port = argv[1];
    }

    if (argc >= 3) {
        socket_port = std::stoi(argv[2]);
    }

    try {
        std::cout << "Connecting to " << serial_port << "...\n";

        clearpath::Transport::instance().configure(serial_port.c_str(), 3);

        {
            std::lock_guard<std::mutex> lock(husky_mutex);

            clearpath::SetMaxAccel(max_accel, max_accel).send();
            clearpath::SetMaxSpeed(max_speed, max_speed).send();

            for (int i = 0; i < 5; ++i) {
                clearpath::SetDifferentialSpeed(0.0, 0.0, max_accel, max_accel).send();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        std::thread command_thread(husky_command_loop);

        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        server_fd_global = server_fd;

        if (server_fd < 0) {
            throw std::runtime_error("Failed to create socket");
        }

        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(socket_port);

        if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(server_fd);
            throw std::runtime_error("Failed to bind socket");
        }

        if (listen(server_fd, 5) < 0) {
            close(server_fd);
            throw std::runtime_error("Failed to listen on socket");
        }

        std::cout << "A200 server running on port " << socket_port << "\n";
        std::cout << "Sending drive heartbeat every 50 ms\n";
        std::cout << "Velocity timeout: " << COMMAND_TIMEOUT_SEC << " seconds\n";

        while (keep_running) {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);

            int client_fd = accept(
                server_fd,
                reinterpret_cast<sockaddr*>(&client_addr),
                &client_len
            );

            if (client_fd < 0) {
                if (keep_running) {
                    std::cerr << "accept() failed\n";
                }
                continue;
            }

            std::thread(client_loop, client_fd).detach();
        }

        keep_running = false;

        if (command_thread.joinable()) {
            command_thread.join();
        }

        {
            std::lock_guard<std::mutex> lock(husky_mutex);

            for (int i = 0; i < 5; ++i) {
                clearpath::SetDifferentialSpeed(0.0, 0.0, max_accel, max_accel).send();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            clearpath::Transport::instance().close();
        }

        if (server_fd_global >= 0) {
            close(server_fd_global);
            server_fd_global = -1;
        }

        return 0;
    }
    catch (clearpath::Exception* ex) {
        std::cerr << "Clearpath exception: " << ex->message << "\n";
        clearpath::Transport::instance().close();
        return 1;
    }
    catch (const std::exception& ex) {
        std::cerr << "std::exception: " << ex.what() << "\n";
        clearpath::Transport::instance().close();
        return 1;
    }
}
