#pragma once

#include "reactive/Config.hpp"

#include <uhd/convert.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#ifdef __linux__
#    include <boost/filesystem.hpp>
#    include <boost/process.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#endif
#include <chrono>
#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>
#include <numeric>
#include <regex>
#include <thread>

#include <condition_variable>
#include <mutex>
#include <atomic>

#include <type_traits>
#include <typeinfo> 

#include <stdio.h>
#include "ipp.h"
#include "math.h"
#include <cmath>

#include <random>

using namespace std::chrono_literals;

namespace po = boost::program_options;

static bool overflow_message = true;

// Include mutexes
std::mutex recv_mutex;
std::mutex signal_mutex;

// Atomic boolean variables to ensure thread safety
std::atomic<bool> stop_signal_called{false};
std::atomic<bool> signal_received{false};
std::atomic<bool> tx_from_file{true};

// Condition variable to control thread operation
std::condition_variable RX_done_cv;
std::condition_variable TX_done_cv;

std::vector<double> freqs;

void sig_int_handler(int)
{
    stop_signal_called = true;
}

enum class Technique
{
    CW,
    Multi, 
    Chirp,
};
