#include "simulation/simulation_interface.hpp"
#include <iostream>
#include <limits>

using namespace interface;
using namespace std::chrono_literals;
int failures = 0;
void Check(bool ok, const char* name) {
    std::cout << (ok ? "PASS " : "FAIL ") << name << '\n';
    if (!ok) ++failures;
}
int main() {
    const int mock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (mock < 0 || bind(mock, reinterpret_cast<sockaddr*>(&address), sizeof(address))) return 2;
    socklen_t length = sizeof(address);
    getsockname(mock, reinterpret_cast<sockaddr*>(&address), &length);
    SimulationInterface interface("test", 12, 0, ntohs(address.sin_port), 0.03);
    interface.Start();
    sockaddr_in observation = address;
    observation.sin_port = htons(interface.observation_port());
    pollfd descriptor{mock, POLLIN, 0};
    Check(poll(&descriptor, 1, 10) == 0, "no zero command sent before first controller command");

    auto publish = [&](int index) {
        std::array<char, 188> bytes;
        const double time = index * 0.001;
        std::array<float, 45> values;
        for (int j = 0; j < 45; ++j) values[j] = index * 100 + j;
        std::memcpy(bytes.data(), &time, 8);
        std::memcpy(bytes.data() + 8, values.data(), 180);
        sendto(mock, bytes.data(), bytes.size(), 0, reinterpret_cast<sockaddr*>(&observation), sizeof(observation));
    };
    std::atomic<bool> publishing{true};
    std::thread publisher([&] {
        for (int i = 1; i <= 300; ++i) { publish(i); std::this_thread::sleep_for(150us); }
        publishing = false;
    });
    bool coherent = true;
    RobotStateSnapshot state;
    std::uint64_t previous = 0;
    while (publishing) {
        if (interface.ReadState(&state)) {
            coherent = coherent && state.sequence >= previous
                && state.q(0) - state.rpy(0) == 9 && state.dq(0) - state.rpy(0) == 21
                && state.tau(0) - state.rpy(0) == 33 && state.received_wall_ns > 0;
            previous = state.sequence;
        }
        std::this_thread::yield();
    }
    publisher.join();
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    do {
        interface.ReadState(&state);
        if (state.time >= 0.300) break;
        std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    Check(coherent && state.time >= 0.300, "whole-packet state coherence under concurrent publication");
    const auto sequence = state.sequence;
    publish(100); publish(300);
    const char bad = 0;
    sendto(mock, &bad, 1, 0, reinterpret_cast<sockaddr*>(&observation), sizeof(observation));
    std::this_thread::sleep_for(10ms);
    interface.ReadState(&state);
    Check(state.sequence == sequence, "duplicate, out-of-order and short packets rejected");

    Eigen::Matrix<float, 12, 5> command; command.setConstant(1.25f);
    interface.SetJointCommand(command);
    std::array<float, 60> packet;
    bool found_command = false, found_watchdog = false;
    const auto command_deadline = std::chrono::steady_clock::now() + 200ms;
    while (std::chrono::steady_clock::now() < command_deadline) {
        descriptor.revents = 0;
        if (poll(&descriptor, 1, 20) <= 0) continue;
        if (recv(mock, packet.data(), sizeof(packet), 0) != sizeof(packet)) continue;
        const Eigen::Map<Eigen::Matrix<float, 12, 5>> received(packet.data());
        found_command = found_command || (received.array() == 1.25f).all();
        found_watchdog = found_watchdog || (received.col(0).isZero()
            && (received.col(2).array() == 2.5f).all() && received.col(4).isZero());
        if (found_command && found_watchdog) break;
    }
    Check(found_command, "unchanged 240-byte column-major command protocol");
    Check(found_watchdog, "stale controller command replaced by damping");
    bool rejected = false;
    command(0, 0) = std::numeric_limits<float>::quiet_NaN();
    try { interface.SetJointCommand(command); }
    catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected, "nonfinite command rejected");
    const auto stop_start = std::chrono::steady_clock::now();
    interface.Stop(); interface.Stop();
    Check(std::chrono::steady_clock::now() - stop_start < 300ms,
          "Stop bounded without simulation traffic, idempotent");
    interface.Start();
    RobotStateSnapshot reset;
    Check(!interface.ReadState(&reset), "restart discards previous observation epoch");
    interface.Stop();
    close(mock);
    return failures ? 1 : 0;
}
