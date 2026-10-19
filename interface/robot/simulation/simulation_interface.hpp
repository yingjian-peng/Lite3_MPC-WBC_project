#pragma once

#include "robot_interface.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace interface {
class SimulationInterface : public RobotInterface {
public:
    explicit SimulationInterface(const std::string& name, int dof_num = 12,
                                  int observation_port = 30010, int command_port = 20001,
                                  double command_timeout = 0.1)
        : RobotInterface(name, dof_num), observation_port_(observation_port),
          command_port_(command_port), command_timeout_(command_timeout) {
        if (dof_num != 12 || observation_port < 0 || observation_port > 65535
            || command_port < 1 || command_port > 65535
            || !std::isfinite(command_timeout) || command_timeout <= 0)
            throw std::invalid_argument("invalid Lite3 simulation interface configuration");
        joint_cmd_ = MatXf::Zero(12, 5);
    }
    ~SimulationInterface() override { Stop(); }

    bool ReadState(RobotStateSnapshot* out) override {
        if (!out) return false;
        std::lock_guard<std::mutex> guard(state_mutex_);
        *out = state_;
        return state_.sequence != 0;
    }
    double GetInterfaceTimeStamp() override { return CopyState().time; }
    VecXf GetJointPosition() override { return CopyState().q; }
    VecXf GetJointVelocity() override { return CopyState().dq; }
    VecXf GetJointTorque() override { return CopyState().tau; }
    Vec3f GetImuRpy() override { return CopyState().rpy; }
    Vec3f GetImuOmega() override { return CopyState().omega_body; }
    Vec3f GetImuAcc() override { return CopyState().acc_body; }
    VecXf GetContactForce() override { return VecXf::Zero(4); }
    MatXf GetJointCommand() override {
        std::lock_guard<std::mutex> guard(command_mutex_);
        return joint_cmd_;
    }
    void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) override {
        if (input.rows() != 12 || !input.allFinite()
            || (input.col(0).array() < 0).any() || (input.col(2).array() < 0).any())
            throw std::invalid_argument("invalid Lite3 joint command");
        std::lock_guard<std::mutex> guard(command_mutex_);
        joint_cmd_ = input;
        command_time_ = std::chrono::steady_clock::now();
        have_command_ = true;
    }

    // Lifecycle calls are made by the owner, not concurrently with Start/Stop.
    void Start() override {
        if (running_) return;
        receive_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        send_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = htons(observation_port_);
        if (receive_socket_ < 0 || send_socket_ < 0
            || bind(receive_socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
            CloseSockets();
            throw std::runtime_error("cannot bind Lite3 simulation observation socket");
        }
        socklen_t length = sizeof(address);
        getsockname(receive_socket_, reinterpret_cast<sockaddr*>(&address), &length);
        observation_port_ = ntohs(address.sin_port);
        destination_ = {};
        destination_.sin_family = AF_INET;
        destination_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        destination_.sin_port = htons(command_port_);
        {
            std::lock_guard<std::mutex> guard(state_mutex_);
            state_ = RobotStateSnapshot{};
        }
        {
            std::lock_guard<std::mutex> guard(command_mutex_);
            have_command_ = false;
        }
        running_ = true;
        try {
            receiver_ = std::thread(&SimulationInterface::ReceiveLoop, this);
            sender_ = std::thread(&SimulationInterface::SendLoop, this);
        } catch (...) {
            Stop();
            throw;
        }
    }
    void Stop() override {
        if (!running_.exchange(false)) return;
        if (receiver_.joinable()) receiver_.join();
        if (sender_.joinable()) sender_.join();
        // Do not leave the simulator executing an old high-gain or torque packet.
        // The simulator also has its own watchdog in case this UDP packet is lost.
        if (CopyState().sequence != 0) Send(DampingCommand());
        CloseSockets();
    }
    int observation_port() const { return observation_port_; }

private:
    using FixedCommand = Eigen::Matrix<float, 12, 5>;
    RobotStateSnapshot CopyState() {
        RobotStateSnapshot out;
        ReadState(&out);
        return out;
    }
    FixedCommand DampingCommand() const {
        FixedCommand cmd = FixedCommand::Zero();
        cmd.col(2).setConstant(2.5f);
        return cmd;
    }
    void CloseSockets() {
        if (receive_socket_ >= 0) close(receive_socket_);
        if (send_socket_ >= 0) close(send_socket_);
        receive_socket_ = send_socket_ = -1;
    }
    void Send(const FixedCommand& cmd) {
        sendto(send_socket_, cmd.data(), sizeof(float) * 60, 0,
               reinterpret_cast<sockaddr*>(&destination_), sizeof(destination_));
    }
    void ReceiveLoop() {
        // Wire protocol remains double time + 45 floats. memcpy avoids alignment
        // and strict-aliasing violations from treating a float array as a double.
        std::array<char, 189> bytes{};
        while (running_) {
            pollfd descriptor{receive_socket_, POLLIN, 0};
            if (poll(&descriptor, 1, 50) <= 0 || !(descriptor.revents & POLLIN)) continue;
            const auto size = recv(receive_socket_, bytes.data(), bytes.size(), MSG_DONTWAIT);
            if (size != 188) continue;
            RobotStateSnapshot next;
            std::array<float, 45> values;
            std::memcpy(&next.time, bytes.data(), sizeof(double));
            std::memcpy(values.data(), bytes.data() + sizeof(double), sizeof(float) * 45);
            if (!std::isfinite(next.time) || next.time < 0) continue;
            bool finite = true;
            for (float value : values) finite = finite && std::isfinite(value);
            if (!finite) continue;
            next.rpy = Eigen::Map<Vec3f>(values.data());
            next.acc_body = Eigen::Map<Vec3f>(values.data() + 3);
            next.omega_body = Eigen::Map<Vec3f>(values.data() + 6);
            next.q = Eigen::Map<Eigen::Matrix<float, 12, 1>>(values.data() + 9);
            next.dq = Eigen::Map<Eigen::Matrix<float, 12, 1>>(values.data() + 21);
            next.tau = Eigen::Map<Eigen::Matrix<float, 12, 1>>(values.data() + 33);
            next.received_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            std::lock_guard<std::mutex> guard(state_mutex_);
            if (state_.sequence && next.time <= state_.time) continue;
            next.sequence = state_.sequence + 1;
            state_ = next;
        }
    }
    void SendLoop() {
        const auto period = std::chrono::microseconds(1000);
        auto tick = std::chrono::steady_clock::now();
        while (running_) {
            FixedCommand command;
            bool ready;
            {
                std::lock_guard<std::mutex> guard(command_mutex_);
                ready = have_command_;
                command = joint_cmd_;
                if (ready && std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - command_time_).count() > command_timeout_)
                    command = DampingCommand();
            }
            if (ready) Send(command);
            tick += period;
            const auto now = std::chrono::steady_clock::now();
            if (tick < now) tick = now + period; // no catch-up packet burst
            std::this_thread::sleep_until(tick);
        }
    }
    std::mutex state_mutex_, command_mutex_;
    RobotStateSnapshot state_;
    bool have_command_ = false;
    std::chrono::steady_clock::time_point command_time_;
    std::atomic<bool> running_{false};
    std::thread receiver_, sender_;
    int receive_socket_ = -1, send_socket_ = -1;
    int observation_port_, command_port_;
    double command_timeout_;
    sockaddr_in destination_{};
};
}  // namespace interface
