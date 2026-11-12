// Standalone SDK timing experiment: no VR, IK, callbacks or query threads.
#include "aubo_sdk/serviceinterface.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sched.h>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
struct Sample {
    double start_s, sdk_ms, wait_ms, iteration_ms;
    bool waited;
    int rc;
};
void print_stats(const char* name, std::vector<double> values) {
    if (values.empty()) return;
    std::sort(values.begin(), values.end());
    const auto percentile = [&](double fraction) {
        return values[static_cast<std::size_t>(std::ceil(fraction * values.size())) - 1];
    };
    std::cout << name << "_mean_ms="
              << std::accumulate(values.begin(), values.end(), 0.0) / values.size() << "\n"
              << name << "_min_ms=" << values.front() << "\n"
              << name << "_p50_ms=" << percentile(.5) << "\n"
              << name << "_p95_ms=" << percentile(.95) << "\n"
              << name << "_p99_ms=" << percentile(.99) << "\n"
              << name << "_max_ms=" << values.back() << "\n";
}
}

int main(int argc, char** argv) {
    bool execute = false;
    std::string host = "192.168.1.5";
    std::string timing_csv;
    int period_us = 5000;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--execute") execute = true;
        else if (arg == "--host" && i + 1 < argc) host = argv[++i];
        else if (arg == "--timing-csv" && i + 1 < argc) timing_csv = argv[++i];
        else if (arg == "--period-us" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value != "5000" && value != "8000" && value != "10000") {
                std::cerr << "Period must be 5000, 8000 or 10000 microseconds\n";
                return 2;
            }
            period_us = std::stoi(value);
        }
        else {
            std::cerr << "Usage: tcp2can_minimal [--execute] [--host IP] [--timing-csv PATH] [--period-us 5000|8000|10000]\n";
            return 2;
        }
    }
    if (!execute) {
        std::cout << "Plan only; no SDK connection. Use --execute to run.\n"
                  << "Read current waypoint once, enter TCP2CAN, then 1000 sends.\n"
                  << "Each send: J6 += 0.0001 rad; time SetRobotPosData2Canbus.\n"
                  << "If SDK returns before t0+" << period_us << "us, sleep_until that deadline; otherwise no sleep.\n"
                  << "Total requested J6 increment: 0.1 rad (5.72958 degrees).\n";
        return 0;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    sched_param scheduling{};
    const int scheduler_policy = sched_getscheduler(0);
    const int scheduler_rc = sched_getparam(0, &scheduling);
    std::cout << "period_us=" << period_us << "\n"
              << "scheduler_policy=" << scheduler_policy << "\n"
              << "scheduler_priority=" << (scheduler_rc == 0 ? scheduling.sched_priority : -1) << std::endl;
    const char* password = std::getenv("AUBO_SDK_PASSWORD");
    // Same default as the supplied official SDK examples; never print it.
    if (!password) password = "123456";
    ServiceInterface robotService;
    int rc = robotService.robotServiceLogin(host.c_str(), 8899, "aubo", password);
    std::cout << "login_rc=" << rc << std::endl;
    if (rc != 0) return 1;

    aubo_robot_namespace::RobotState state{};
    rc = robotService.robotServiceGetRobotCurrentState(state);
    std::cout << "pretest_state_rc=" << rc << ", robot_state=" << int(state) << std::endl;
    if (rc != 0 || state != aubo_robot_namespace::RobotStopped) {
        std::cerr << "Test requires a stopped robot\n";
        robotService.robotServiceLogout();
        return 1;
    }

    aubo_robot_namespace::wayPoint_S current_wp{};
    rc = robotService.robotServiceGetCurrentWaypointInfo(current_wp);
    std::cout << "get_current_waypoint_rc=" << rc << std::endl;
    if (rc != 0) {
        robotService.robotServiceLogout();
        return 1;
    }
    double q[6];
    std::cout << std::setprecision(12) << "initial_q_rad=";
    for (int i = 0; i < 6; ++i) {
        q[i] = current_wp.jointpos[i];
        if (!std::isfinite(q[i])) {
            std::cerr << "Invalid current joint angle\n";
            robotService.robotServiceLogout();
            return 1;
        }
        std::cout << (i ? "," : "") << q[i];
    }
    std::cout << std::endl;
    if (interrupted) {
        robotService.robotServiceLogout();
        return 1;
    }
    rc = robotService.robotServiceEnterTcp2CanbusMode();
    std::cout << "enter_tcp2can_rc=" << rc << std::endl;
    int completed = 0;
    int send_rc = 0;
    std::vector<Sample> samples;
    samples.reserve(1000);
    const auto start = std::chrono::steady_clock::now();
    if (rc == 0) {
        for (int step = 0; step < 1000 && !interrupted; ++step) {
            q[5] += 0.0001;
            const auto t0 = std::chrono::steady_clock::now();
            // The only SDK call inside the timed loop.
            send_rc = robotService.robotServiceSetRobotPosData2Canbus(q);
            const auto t1 = std::chrono::steady_clock::now();
            const auto target_wake_time = t0 + std::chrono::microseconds(period_us);
            const bool waited = send_rc == 0 && !interrupted && t1 < target_wake_time;
            if (waited) std::this_thread::sleep_until(target_wake_time);
            const auto t2 = std::chrono::steady_clock::now();
            samples.push_back({std::chrono::duration<double>(t0 - start).count(),
                std::chrono::duration<double, std::milli>(t1 - t0).count(),
                std::chrono::duration<double, std::milli>(t2 - t1).count(),
                std::chrono::duration<double, std::milli>(t2 - t0).count(), waited, send_rc});
            if (send_rc != 0) break;
            ++completed;
        }
    }
    const auto end = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(end - start).count();

    // Also attempt leave if entry returned an error with an uncertain outcome.
    const int leave_rc = robotService.robotServiceLeaveTcp2CanbusMode();
    const int logout_rc = robotService.robotServiceLogout();
    std::cout << "successful_sends=" << completed << "\n"
              << "send_rc=" << send_rc << "\n"
              << "elapsed_s=" << seconds << "\n"
              << "actual_frequency_hz=" << (seconds > 0 ? completed / seconds : 0) << "\n"
              << "last_requested_j6_rad=" << q[5] << "\n"
              << "interrupted=" << interrupted << "\n"
              << "leave_tcp2can_rc=" << leave_rc << "\n"
              << "logout_rc=" << logout_rc << std::endl;
    std::vector<double> sdk_costs, start_intervals, waits;
    int over_period = 0;
    int over_5ms = 0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        sdk_costs.push_back(samples[i].sdk_ms);
        over_period += samples[i].sdk_ms > period_us / 1000.;
        over_5ms += samples[i].sdk_ms > 5.;
        if (samples[i].waited) waits.push_back(samples[i].wait_ms);
        if (i) start_intervals.push_back(1000. * (samples[i].start_s - samples[i-1].start_s));
    }
    print_stats("sdk", sdk_costs);
    print_stats("send_start_interval", start_intervals);
    print_stats("conditional_wait", waits);
    std::cout << "sdk_over_5ms_count=" << over_5ms << "\n"
              << "sdk_over_requested_period_count=" << over_period << "\n"
              << "conditional_sleep_count=" << waits.size() << std::endl;
    if (!timing_csv.empty()) {
        std::ofstream out(timing_csv);
        out << std::setprecision(17)
            << "step,start_s,sdk_ms,wait_ms,iteration_ms,slept,return_code\n";
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const auto& sample = samples[i];
            out << i << ',' << sample.start_s << ',' << sample.sdk_ms << ',' << sample.wait_ms
                << ',' << sample.iteration_ms << ',' << sample.waited << ',' << sample.rc << '\n';
        }
        out.close();
        if (!out) {
            std::cerr << "Timing CSV write failed: " << timing_csv << std::endl;
            return 1;
        }
        std::cout << "timing_csv=" << timing_csv << std::endl;
    }
    return rc == 0 && completed == 1000 && send_rc == 0 && leave_rc == 0 && logout_rc == 0 ? 0 : 1;
}
