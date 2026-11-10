#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <stdexcept>
#include <iostream>
#include "../include/clearpath.h"

namespace py = pybind11;

const unsigned int SAFETY_TIMEOUT = 0x1;
const unsigned int SAFETY_LOCKOUT = 0x2;
const unsigned int SAFETY_ESTOP   = 0x8;
const unsigned int SAFETY_CCI     = 0x10;
const unsigned int SAFETY_PSU     = 0x20;
const unsigned int SAFETY_CURRENT = 0x40;

class Husky {
public:
    explicit Husky(const std::string& port) {
        connect(port);

        // Subscribe to useful streams
        clearpath::DataSystemStatus::subscribe(10);
        clearpath::DataPowerSystem::subscribe(10);
        clearpath::DataSafetySystemStatus::subscribe(10);
        clearpath::DataVelocity::subscribe(10);
        clearpath::DataDifferentialSpeed::subscribe(10);
    }

    ~Husky() {
        try {
            disconnect();
        } catch (...) {}
    }

    void connect(const std::string& port) {
        if (connected_) {
            disconnect();
        }
        clearpath::Transport::instance().configure(port.c_str(), 3);
        connected_ = true;
    }

    void disconnect() {
        if (connected_) {
            clearpath::Transport::instance().close();
            connected_ = false;
        }
    }

    bool is_connected() const {
        return connected_;
    }

    void set_diff_speed(double left_speed, double right_speed,
                        double left_accel, double right_accel) {
        ensure_connected();
        clearpath::SetDifferentialSpeed(left_speed, right_speed, left_accel, right_accel).send();
    }

    void set_velocity(double translational, double rotational, double accel) {
        ensure_connected();
        clearpath::SetVelocity(translational, rotational, accel).send();
    }

    void set_ackermann_output(double steering, double throttle, double brake) {
        ensure_connected();
        clearpath::SetAckermannOutput(steering, throttle, brake).send();
    }

    void reset_processor() {
        ensure_connected();
        clearpath::CmdProcessorReset().send();
    }

    py::dict get_firmware_info(double timeout = 1.0) {
        ensure_connected();
        py::dict out;

        auto* info = clearpath::DataFirmwareInfo::getUpdate(timeout);
        if (!info) {
            throw std::runtime_error("Timed out waiting for firmware info");
        }

        out["firmware_version"] = std::to_string(info->getMajorFirmwareVersion()) + "." + std::to_string(info->getMinorFirmwareVersion());

        out["protocol_version"] = std::to_string(info->getMajorProtocolVersion()) + "." + std::to_string(info->getMinorProtocolVersion());

        delete info;
        return out;
    }

    py::dict get_system_status(double timeout = 1.0) {
        ensure_connected();
        py::dict out;

        auto* status = clearpath::DataSystemStatus::getUpdate(timeout);
        if (!status) {
            throw std::runtime_error("Timed out waiting for system status");
        }

        out["uptime"] = status->getUptime();

        py::list voltages;
        for (uint8_t i = 0; i < status->getVoltagesCount(); ++i) {
            voltages.append(status->getVoltage(i));
        }
        out["voltages"] = voltages;

        py::list currents;
        for (uint8_t i = 0; i < status->getCurrentsCount(); ++i) {
            currents.append(status->getCurrent(i));
        }
        out["currents"] = currents;

        py::list temperatures;
        for (uint8_t i = 0; i < status->getTemperaturesCount(); ++i) {
            temperatures.append(status->getTemperature(i));
        }
        out["temperatures"] = temperatures;

        delete status;
        return out;
    }

    py::dict get_power_status(double timeout = 1.0) {
        ensure_connected();
        py::dict out;

        auto* power = clearpath::DataPowerSystem::getUpdate(timeout);
        if (!power) {
            throw std::runtime_error("Timed out waiting for power status");
        }

        py::list batteries;
        for (uint8_t i = 0; i < power->getBatteryCount(); ++i) {
            py::dict b;

            auto desc = power->getDescription(i);
            b["index"] = i;
            b["present"] = desc.isPresent();
            b["in_use"] = desc.isInUse();
            b["type"] = static_cast<int>(desc.getType());
            b["charge_estimate"] = power->getChargeEstimate(i);
            b["capacity_estimate"] = power->getCapacityEstimate(i);

            batteries.append(b);
        }

        out["battery_count"] = power->getBatteryCount();
        out["batteries"] = batteries;

        delete power;
        return out;
    }

    py::dict get_safety_status(double timeout = 1.0) {
        ensure_connected();
        py::dict out;

        auto* safety = clearpath::DataSafetySystemStatus::getUpdate(timeout);
        if (!safety) {
            throw std::runtime_error("Timed out waiting for safety status");
        }

        uint16_t flags = safety->getFlags();
        out["flags"] = flags;
        out["timeout"] = static_cast<bool>(flags & SAFETY_TIMEOUT);
        out["lockout"] = static_cast<bool>(flags & SAFETY_LOCKOUT);
        out["estop"] = static_cast<bool>(flags & SAFETY_ESTOP);
        out["cci"] = static_cast<bool>(flags & SAFETY_CCI);
        out["psu"] = static_cast<bool>(flags & SAFETY_PSU);
        out["current_limit"] = static_cast<bool>(flags & SAFETY_CURRENT);

        delete safety;
        return out;
    }

    bool get_estop(double timeout = 1.0) {
        auto safety = get_safety_status(timeout);
        return py::cast<bool>(safety["estop"]);
    }

    py::dict get_velocity(double timeout = 1.0) {
        ensure_connected();
        py::dict out;

        auto* vel = clearpath::DataVelocity::getUpdate(timeout);
        if (!vel) {
            throw std::runtime_error("Timed out waiting for velocity");
        }

        out["translational"] = vel->getTranslational();
        out["rotational"] = vel->getRotational();
        out["trans_accel"] = vel->getTransAccel();

        delete vel;
        return out;
    }

    py::dict get_encoder_data(double timeout = 1.0) {
        ensure_connected();
        py::dict out;

        auto* enc = clearpath::DataEncoders::getUpdate(timeout);
        if (!enc) {
            throw std::runtime_error("Timed out waiting for encoder data");
        }

        py::list travel;
        py::list speed;
        uint8_t count = enc->getCount();

        for (uint8_t i = 0; i < count; ++i) {
            travel.append(enc->getTravel(i));
            speed.append(enc->getSpeed(i));
        }

        out["count"] = count;
        out["travel"] = travel;
        out["speed"] = speed;

        delete enc;
        return out;
    }

    py::dict get_encoder_raw(double timeout = 1.0) {
        ensure_connected();
        py::dict out;

        auto* enc = clearpath::DataEncodersRaw::getUpdate(timeout);
        if (!enc) {
            throw std::runtime_error("Timed out waiting for raw encoder data");
        }

        py::list ticks;
        uint8_t count = enc->getCount();

        for (uint8_t i = 0; i < count; ++i) {
            ticks.append(enc->getTicks(i));
        }

        out["count"] = count;
        out["ticks"] = ticks;

        delete enc;
        return out;
    }

    py::dict get_differential_speed(double timeout = 1.0) {
        ensure_connected();
        py::dict out;

        auto* spd = clearpath::DataDifferentialSpeed::getUpdate(timeout);
        if (!spd) {
            throw std::runtime_error("Timed out waiting for differential speed");
        }

        out["left_speed"] = spd->getLeftSpeed();
        out["right_speed"] = spd->getRightSpeed();
        out["left_accel"] = spd->getLeftAccel();
        out["right_accel"] = spd->getRightAccel();

        delete spd;
        return out;
    }

private:
    bool connected_ = false;

    void ensure_connected() const {
        if (!connected_) {
            throw std::runtime_error("Husky is not connected");
        }
    }
};

PYBIND11_MODULE(a200_serial_pybind, m) {
    m.doc() = "Pybind11 wrapper for Clearpath Horizon legacy API";

    m.attr("SAFETY_TIMEOUT") = py::int_(SAFETY_TIMEOUT);
    m.attr("SAFETY_LOCKOUT") = py::int_(SAFETY_LOCKOUT);
    m.attr("SAFETY_ESTOP") = py::int_(SAFETY_ESTOP);
    m.attr("SAFETY_CCI") = py::int_(SAFETY_CCI);
    m.attr("SAFETY_PSU") = py::int_(SAFETY_PSU);
    m.attr("SAFETY_CURRENT") = py::int_(SAFETY_CURRENT);

    py::class_<Husky>(m, "Husky")
        .def(py::init<const std::string&>(), py::arg("port"))
        .def("connect", &Husky::connect, py::arg("port"))
        .def("disconnect", &Husky::disconnect)
        .def("is_connected", &Husky::is_connected)
        .def("set_diff_speed", &Husky::set_diff_speed,
             py::arg("left_speed"), py::arg("right_speed"),
             py::arg("left_accel"), py::arg("right_accel"))
        .def("set_velocity", &Husky::set_velocity,
             py::arg("translational"), py::arg("rotational"), py::arg("accel"))
        .def("set_ackermann_output", &Husky::set_ackermann_output,
             py::arg("steering"), py::arg("throttle"), py::arg("brake"))
        .def("reset_processor", &Husky::reset_processor)
        .def("get_firmware_info", &Husky::get_firmware_info, py::arg("timeout") = 1.0)
        .def("get_system_status", &Husky::get_system_status, py::arg("timeout") = 1.0)
        .def("get_power_status", &Husky::get_power_status, py::arg("timeout") = 1.0)
        .def("get_safety_status", &Husky::get_safety_status, py::arg("timeout") = 1.0)
        .def("get_estop", &Husky::get_estop, py::arg("timeout") = 1.0)
        .def("get_velocity", &Husky::get_velocity, py::arg("timeout") = 1.0)
        .def("get_encoder_data", &Husky::get_encoder_data, py::arg("timeout") = 1.0)
        .def("get_encoder_data_raw", &Husky::get_encoder_raw, py::arg("timeout") = 1.0)
        .def("get_differential_speed", &Husky::get_differential_speed, py::arg("timeout") = 1.0);
}
