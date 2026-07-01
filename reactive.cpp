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

#define WOLA_ORDER 4
#define JAM_DUR 10
#define NOISE_BW 100e3

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

class USRPconfig 
{
    public:
        using get_sensor_fn_t = std::function<uhd::sensor_value_t(const std::string&)>;

        USRPconfig(uhd::usrp::multi_usrp::sptr usrp) : usrp_(usrp) {}

        bool check_locked_sensor(std::vector<std::string> sensor_names,
            const char* sensor_name,
            get_sensor_fn_t get_sensor_fn,
            double setup_time)
        {
            if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name)
                == sensor_names.end())
                return false;

            const auto setup_timeout = std::chrono::steady_clock::now() + (setup_time * 1s);
            bool lock_detected       = false;

            std::cout << "Waiting for \"" << sensor_name << "\": ";
            std::cout.flush();

            while (true) {
                if (lock_detected and (std::chrono::steady_clock::now() > setup_timeout)) {
                    std::cout << " locked." << std::endl;
                    break;
                }
                if (get_sensor_fn(sensor_name).to_bool()) {
                    std::cout << "+";
                    std::cout.flush();
                    lock_detected = true;
                } else {
                    if (std::chrono::steady_clock::now() > setup_timeout) {
                        std::cout << std::endl;
                        throw std::runtime_error(
                            str(boost::format(
                                    "timed out waiting for consecutive locks on sensor \"%s\"")
                                % sensor_name));
                    }
                    std::cout << "_";
                    std::cout.flush();
                }
                std::this_thread::sleep_for(100ms);
            }
            std::cout << std::endl;
            return true;
        }

        void parse_channels(std::vector<std::string>& channel_strings,
            const std::string& channels,
            size_t max_channels,
            std::vector<size_t>& channel_list)
        {
            boost::split(channel_strings, channels, boost::is_any_of("\"',"));

            for (const auto& ch_str : channel_strings) {
                try {
                    int chan = std::stoi(ch_str);
                    if (chan < 0 || chan >= static_cast<int>(max_channels)) {
                        throw std::runtime_error("Invalid channel(s) specified.");
                    }
                    channel_list.push_back(static_cast<size_t>(chan));
                } catch (const std::invalid_argument& c) {
                    throw std::runtime_error("Invalid channel(s) specified.");
                } catch (const std::out_of_range& c) {
                    throw std::runtime_error("Invalid channel(s) specified.");
                }
            }
        }

        void set_channel_rate(double rate, bool is_rx,
            const std::vector<size_t>& channels)
        {
            if (is_rx) {
                std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate / 1e6) << std::endl;
                usrp_->set_rx_rate(rate, uhd::usrp::multi_usrp::ALL_CHANS);
                std::cout << boost::format("Actual RX Rate: %f Msps...")
                            % (usrp_->get_rx_rate(channels[0]) / 1e6)
                    << std::endl
                    << std::endl;
            } else {
                std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate / 1e6) << std::endl;
                usrp_->set_tx_rate(rate, uhd::usrp::multi_usrp::ALL_CHANS);
                std::cout << boost::format("Actual TX Rate: %f Msps...")
                            % (usrp_->get_tx_rate(channels[0]) / 1e6)
                    << std::endl
                    << std::endl;
            }
        }

        void set_channel_freq(double freq,
            double lo_offset,
            bool is_rx,
            bool int_n,
            const std::vector<size_t>& channels)
        {
            std::string dir = is_rx ? "RX" : "TX";
            std::cout << boost::format("Setting %s freq: %.3f MHz...\n") % dir % (freq / 1e6);

            uhd::tune_request_t tune_request(freq, lo_offset);
            if (int_n)
                tune_request.args = uhd::device_addr_t("mode_n=integer");

            for (auto chan : channels) {
                if (is_rx)
                    usrp_->set_rx_freq(tune_request, chan);
                else
                    usrp_->set_tx_freq(tune_request, chan);
            }
            std::cout << "[set_channel_freq] TX freq: " << usrp_->get_tx_freq(channels[0]) / 1e6 << std::endl;
            double actual_freq = is_rx ? usrp_->get_rx_freq(channels[0]) : usrp_->get_tx_freq(channels[0]);
            std::cout << boost::format("Actual %s freq: %.3f MHz\n") % dir % (actual_freq / 1e6);
        }

        void set_channel_gain(double gain,
            bool is_rx,
            const std::vector<size_t>& channels)
        {
            std::string dir = is_rx ? "RX" : "TX";
            std::cout << boost::format("Setting %s gain: %f dB...") % dir % gain << std::endl;

            for (auto chan : channels) {
                if (is_rx) 
                    usrp_->set_rx_gain(gain, chan);
                else 
                    usrp_->set_tx_gain(gain, chan);
            }

            double actual_gain = is_rx ? usrp_->get_rx_gain(channels[0]) : usrp_->get_tx_gain(channels[0]);
            std::cout << boost::format("Actual %s gain: %f dB\n") % dir % actual_gain;
        }

        void set_channel_bw(double bw,
            bool is_rx,
            const std::vector<size_t>& channels)
        {
            std::string dir = is_rx ? "RX" : "TX";
            std::cout << boost::format("Setting %s Bandwidth: %f MHz...") % dir % (bw / 1e6) << std::endl;

            for (auto chan : channels)
            {
                if (is_rx)
                    usrp_->set_rx_bandwidth(bw, chan);
                else
                    usrp_->set_tx_bandwidth(bw, chan);
            }

            double actual_bw = is_rx ? usrp_->get_rx_bandwidth(channels[0]) : usrp_->get_tx_bandwidth(channels[0]);
            std::cout << boost::format("Actual %s bw: %.3f MHz\n") % dir % (actual_bw / 1e6);
        }

        void set_channel_antenna(const std::string& antenna,
            bool is_rx,
            std::vector<size_t>& channels)
        {
            for (auto chan : channels) {
                if (is_rx) 
                    usrp_->set_rx_antenna(antenna, chan);
                else
                    usrp_->set_tx_antenna(antenna, chan);
            }
        }

        void lock_lo(bool is_rx,
            const std::vector<size_t>& channels,
            double setup_time)
        {
            std::string dir = is_rx ? "RX" : "TX";
            for (size_t chan : channels) {
                std::cout << "Locking " << (is_rx ? "RX" : "TX") << " LO on channel " << chan << std::endl;
                
                auto sensor_names = is_rx
                    ? usrp_->get_rx_sensor_names(chan)
                    : usrp_->get_tx_sensor_names(chan);

                auto get_sensor_fn = [this, chan, is_rx](const std::string& sensor_name) {
                    return is_rx
                        ? usrp_->get_rx_sensor(sensor_name, chan)
                        : usrp_->get_tx_sensor(sensor_name, chan);
                };

                check_locked_sensor(sensor_names, "lo_locked", get_sensor_fn, setup_time);
            }

        }

        private:
            uhd::usrp::multi_usrp::sptr usrp_;
};

class SignalGenerator
{
    public:
        SignalGenerator(uhd::usrp::multi_usrp::sptr usrp,
            const std::string& file,
            size_t samps_per_buff,
            const std::vector<size_t>& tx_channels)
                  : usrp(usrp),
                    file(file),
                    samps_per_buff(samps_per_buff),
                    tx_channels(tx_channels)
        {}

        std::vector<std::complex<short>> create_cw()
        {
            std::vector<double> tx_freqs(tx_channels.size());
            for (size_t i = 0; i < tx_channels.size(); i++)
            {
                tx_freqs[i] = usrp->get_tx_freq(tx_channels[i]);
            }

            const int N = static_cast<int>(samps_per_buff);
            std::vector<std::complex<double>> signal(N, {0.0, 0.0});

            for (size_t i = 0; i < freqs.size(); i++)
            {
                bool skip = false;
                for (int j = 0; j < i; j++) 
                { 
                    if (std::abs(freqs[i] - freqs[j]) <= 6.25e3)
                    { 
                        skip = true; 
                        break; 
                    } 
                }
                    
                if (skip) continue; 

                std::cout << "Creating CW tone for freq " << freqs[i] / 1e6 << " MHz..." << std::endl;

                double baseband = freqs[i] - tx_freqs[i];                
                double dphi = 2.0 * M_PI * baseband / usrp->get_tx_rate();
                double phase = 0.0;

                for (int n = 0; n < N; n++)
                {
                    signal[n] += std::polar(1.0, phase);
                    phase = std::fmod(phase + dphi + M_PI, 2.0 * M_PI) - M_PI;
                }
            }

            // ADD NOISE 
            if (NOISE_BW > 0)
            {
                auto noise = create_gaussian_noise(N);
                for (int n = 0; n < N; n++)
                {
                    signal[n] *= noise[n];
                }
            }
            
            normalize_signal(signal);
            return convert_to_short(signal);
        }    

    private:
        void normalize_signal(std::vector<std::complex<double>>& signal)
        {
            double real_max = 0.0;
            double imag_max = 0.0;

            for (const auto& samp : signal)
            {   
                real_max = std::max(real_max, std::abs(samp.real()));
                imag_max = std::max(imag_max, std::abs(samp.imag()));
            }

            double max = std::max(real_max, imag_max);

            for (auto& samp : signal)
            {
                samp /= max;
            }
        }

        std::vector<std::complex<short>> convert_to_short(
            const std::vector<std::complex<double>>& signal
        )
        {
            std::vector<std::complex<short>> output(signal.size());
            const double scale = 32767.0;

            for (size_t i = 0; i < signal.size(); i++)
            {
                output[i] = std::complex<short>(
                    static_cast<short>(signal[i].real() * scale),
                    static_cast<short>(signal[i].imag() * scale)
                );
            }
            return output;
        }

        std::vector<std::complex<double>> create_gaussian_noise(const size_t num_samps)
        {
            Ipp64fc* noise = ippsMalloc_64fc(num_samps);

            std::mt19937 gen(42);
            std::normal_distribution<double> dist(0.0, 1.0);

            for (size_t n = 0; n < num_samps; n++)            
            {
                noise[n].re = dist(gen);
                noise[n].im = dist(gen);
            }

            double power = 0.0;
            for (size_t n = 0; n < num_samps; n++)            
            {
                power += noise[n].re * noise[n].re +
                         noise[n].im * noise[n].im;
            }
            double rms = std::sqrt(power / num_samps);
            for (size_t n = 0; n < num_samps; n++)            
            {
                noise[n].re /= rms;
                noise[n].im /= rms;
            }

            std::vector<std::complex<double>> noise_out = apply_lpf(noise, num_samps);
            power = 0.0;
            for (size_t n = 0; n < num_samps; n++)
            {
                power += std::norm(noise_out[n]);
            }
            rms = std::sqrt(power / num_samps);

            for (size_t n = 0; n < num_samps; n++)
            {
                noise_out[n] /= rms;
            }

            ippsFree(noise);
            return noise_out;
        }

        std::vector<std::complex<double>> apply_lpf(Ipp64fc* noise, const size_t num_samps)
        {
            const int filter_order = 3;
            double rFreq = NOISE_BW / usrp->get_tx_rate(tx_channels[0]);
            Ipp64f taps[2 * (filter_order + 1)];

            std::cout << "Filter: Noise BW = " << NOISE_BW/1e6 << " MHz," 
                      << "rFreq = " << rFreq << std::endl;

            int bufferSize;
            ippsIIRGenGetBufferSize(filter_order, &bufferSize);
            Ipp8u* pBuffer = ippsMalloc_8u(bufferSize);

            IppStatus status = ippsIIRGenLowpass_64f(rFreq, 0.0, filter_order,
                                                     taps, ippButterworth, pBuffer);
            
            ippsFree(pBuffer);
            if (status != ippStsNoErr)
            {
                ippsFree(noise);
                return std::vector<std::complex<double>>();
            }

            Ipp64f* real_part = ippsMalloc_64f(num_samps);
            Ipp64f* imag_part = ippsMalloc_64f(num_samps);

            for (size_t n = 0; n < num_samps; n++)
            {
                real_part[n] = noise[n].re;
                imag_part[n] = noise[n].im;
            }
            
            int stateSize;
            ippsIIRGetStateSize_64f(filter_order, &stateSize);
            Ipp8u* pStateBufferReal = ippsMalloc_8u(stateSize);
            Ipp8u* pStateBufferImag = ippsMalloc_8u(stateSize);

            IppsIIRState_64f* pStateReal = nullptr;
            IppsIIRState_64f* pStateImag = nullptr;

            status = ippsIIRInit_64f(&pStateReal, taps, filter_order, NULL, pStateBufferReal);
            if (status == ippStsNoErr)
            {
                ippsIIR_64f(real_part, real_part, num_samps, pStateReal);
            }

            status = ippsIIRInit_64f(&pStateImag, taps, filter_order, NULL, pStateBufferImag);
            if (status == ippStsNoErr)
            {
                ippsIIR_64f(imag_part, imag_part, num_samps, pStateImag);
            }
;
            /*
            Ipp8u* pStateBuffer = ippsMalloc_8u(stateSize);
            IppsIIRState_64f* pState = (IppsIIRState_64f*)pStateBuffer;

            ippsIIRInit_64f(&pState, taps, filter_order, NULL, pStateBuffer);
            ippsIIR_64f(real_part, real_part, num_samps, pState);

            ippsIIRInit_64f(&pState, taps, filter_order, NULL, pStateBuffer);
            ippsIIR_64f(imag_part, imag_part, num_samps, pState);
            */
            std::vector<std::complex<double>> noise_out(num_samps);
            for (size_t n = 0; n < num_samps; n++)
            {
                noise_out[n] = std::complex<double>(real_part[n], imag_part[n]);
            }

            ippsFree(pStateBufferReal);
            ippsFree(pStateBufferImag);
            ippsFree(real_part);
            ippsFree(imag_part);

            return noise_out;
        }

        uhd::usrp::multi_usrp::sptr usrp;
        std::vector<size_t> tx_channels;
        size_t samps_per_buff;
        std::string file;
        IppStatus status;
};

template <typename samp_type>
class SignalRecv
{
    public:
        SignalRecv(size_t max_rx_channels, size_t spb, double rate)
                 : samps_per_buff(spb),
                   max_rx_channels(max_rx_channels),
                   fs(rate),
                   sigSize(spb),
                   fftSize(spb / WOLA_ORDER),
                   FFTOrder(static_cast<int>(log2(spb / WOLA_ORDER)))  
        {}

        ~SignalRecv()
        {}

        void samplesRX (uhd::usrp::multi_usrp::sptr usrp,
            uhd::rx_streamer::sptr rx_stream,
            const std::vector<size_t>& rx_channel_nums,
            const size_t total_num_rx_channels,
            size_t samps_per_buff,
            unsigned long long num_requested_samples,
            double& bw,
            bool stats                       = false,
            bool null                        = false,  
            bool enable_size_map             = false,
            bool continue_on_bad_packet      = false,
            const std::string& thread_prefix = "")
        {          
            while (!stop_signal_called) 
            {         
                unsigned long long num_total_samps = 0;
                std::vector<std::vector<samp_type>> received_samps(total_num_rx_channels);
                for(size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
                    received_samps[ch].resize(samps_per_buff); 
                
                uhd::rx_metadata_t md;

                std::vector<samp_type*> buffs(rx_stream->get_num_channels());

                uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
                stream_cmd.stream_now = rx_stream->get_num_channels() == 1;
                stream_cmd.time_spec  = usrp->get_time_now() + uhd::time_spec_t(0.05);
                rx_stream->issue_stream_cmd(stream_cmd);

                typedef std::map<size_t, size_t> SizeMap;
                SizeMap mapSizes;
                const auto start_time = std::chrono::steady_clock::now();
                auto last_update                     = start_time;
                unsigned long long last_update_samps = 0;

                std::cout << "" << std::endl;  
                try {
                    for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++) {
                        buffs[ch] = new samp_type[samps_per_buff];
                    }
                } catch (std::bad_alloc& exc) {
                    UHD_LOGGER_ERROR("UHD")
                        << "Bad memory allocation. "
                        "Try a smaller samples per buffer setting or free up additional memory";
                    std::exit(EXIT_FAILURE);
                }

                while (!stop_signal_called
                       && !signal_received.load()  
                       && (num_requested_samples != num_total_samps || num_requested_samples == 0))  
                {
                    const auto now = std::chrono::steady_clock::now();

                    if (num_total_samps == 0) {
                        for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++) {
                            std::fill(received_samps[ch].begin(), received_samps[ch].end(), samp_type{});
                        }
                    }
                    
                    size_t req_samps = samps_per_buff;
                    if(num_total_samps + req_samps > samps_per_buff)
                    {
                        req_samps = samps_per_buff - num_total_samps;
                    }
                    
                    size_t num_rx_samps =
                        rx_stream->recv(buffs, req_samps, md, 0.1, enable_size_map);

                    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
                    {
                        std::cout << std::endl
                                << thread_prefix << "[RX] Timeout while streaming" << std::endl;
                        break;
                    }
                    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW)
                    {
                        const std::lock_guard<std::mutex> lock(recv_mutex);
                        if (overflow_message)
                        {
                            overflow_message = false;
                            std::cerr
                                << boost::format(
                                    "Got an overflow indication. Please consider the following:\n"
                                    "  Your write medium must sustain a rate of %0.3fMB/s.\n"
                                    "  Dropped samples will not be written to the file.\n"
                                    "  Please modify this example for your purposes.\n"
                                    "  This message will not appear again.\n")
                                    % (usrp->get_rx_rate(rx_channel_nums[0]) * total_num_rx_channels
                                        * sizeof(samp_type) / 1e6);
                        }
                        continue;
                    }
                    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
                    {
                        const std::lock_guard<std::mutex> lock(recv_mutex);
                        std::string error = thread_prefix + "Receiver error: " + md.strerror();
                        if (continue_on_bad_packet)
                        {
                            std::cerr << error << std::endl;
                            continue;
                        } else
                            throw std::runtime_error(error);
                    }

                    if (enable_size_map)
                    {
                        const std::lock_guard<std::mutex> lock(recv_mutex);
                        SizeMap::iterator it = mapSizes.find(num_rx_samps);
                        if (it == mapSizes.end())
                            mapSizes[num_rx_samps] = 0;
                        mapSizes[num_rx_samps] += 1;
                    }
                    
                    for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++) 
                    {
                        memcpy(&received_samps[ch][num_total_samps],
                            buffs[ch],
                            num_rx_samps * sizeof(samp_type));                        
                    }
                    num_total_samps += num_rx_samps;
                    last_update_samps += num_rx_samps;
                    
                    const auto time_since_last_update = now - last_update;
                    if (time_since_last_update > 1s)
                    {
                        const std::lock_guard<std::mutex> lock(recv_mutex);
                        const double time_since_last_update_s =
                            std::chrono::duration<double>(time_since_last_update).count();
                        bw                = double(last_update_samps) / time_since_last_update_s;
                        last_update_samps = 0;
                        last_update       = now;
                    }

                    if(num_total_samps >= samps_per_buff)
                    {
                        freqs = detect_signal(received_samps);
                        num_total_samps = 0;

                        if (freqs.size() > 0) 
                        {
                            std::cout << "[RX] Detected " << freqs.size() << " freqs:" << std::endl;
                            for (size_t i = 0; i < freqs.size(); i++) 
                            {
                                freqs[i] = usrp->get_rx_freq() + freqs[i];
                                std::cout << freqs[i] / 1e6 << " MHz" << std::endl;
                            }
                            std::cout << std::endl;
                            std::cout << std::endl;
                            {
                                std::lock_guard<std::mutex> lock(signal_mutex);
                                signal_received.store(true);
                            }
                            RX_done_cv.notify_one();
                        } 
                    }
                }
                
                stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
                rx_stream->issue_stream_cmd(stream_cmd);

                uhd::rx_metadata_t flush_md;
                std::vector<samp_type> flush_buff(samps_per_buff);
                std::vector<samp_type*> flush_buffs(rx_stream->get_num_channels());
                for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
                {
                    flush_buffs[ch] = flush_buff.data();
                }

                while (true)
                {
                    size_t num_flushed = rx_stream->recv(flush_buffs, samps_per_buff, flush_md, 0.1);
                    if (flush_md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT)
                        break;
                }

                const auto actual_stop_time = std::chrono::steady_clock::now();
                if (stats)
                {
                    const std::lock_guard<std::mutex> lock(recv_mutex);
                    std::cout << std::endl;
                    const double actual_duration_seconds =
                        std::chrono::duration<float>(actual_stop_time - start_time).count();
                    std::cout << boost::format("%sReceived %d samples in %f seconds") % thread_prefix
                                    % num_total_samps % actual_duration_seconds
                            << std::endl;

                    if (enable_size_map) {
                        std::cout << std::endl;
                        std::cout << "Packet size map (bytes: count)" << std::endl;
                        for (SizeMap::iterator it = mapSizes.begin(); it != mapSizes.end(); it++)
                            std::cout << it->first << ":\t" << it->second << std::endl;
                    }
                }
           
                for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
                {
                    delete[] buffs[ch];
                    received_samps[ch].assign(samps_per_buff, samp_type{});
                }

                {
                    std::unique_lock<std::mutex> lock(signal_mutex);
                    TX_done_cv.wait(lock, [] { return !signal_received.load(); });
                }
                
                std::cout << "[RX] Restarting RX..." << std::endl;  
                freqs.clear();
                num_total_samps = 0;  
            }
        }

        std::vector<double> detect_signal(const std::vector<std::vector<samp_type>>& recvd_samps)
        {
            ippSrc.resize(recvd_samps.size());
            for (size_t ch = 0; ch < recvd_samps.size(); ch++) {
                ippSrc[ch].resize(recvd_samps[ch].size());
            }
            
            freqs.clear();
            std::vector<double> freq_list;
            adjust_samps_type(recvd_samps, ippSrc);
            std::cout << "looking for signal.." << std::endl;
            freq_list = compute_FFT(ippSrc);
            return freq_list;
        }

    private:
        void adjust_samps_type(const std::vector<std::vector<samp_type>>& recvd_samps,
            std::vector<std::vector<Ipp32fc>>& ipp32fc_signal)
        {
            ipp32fc_signal.resize(recvd_samps.size());
            for (size_t ch = 0; ch < recvd_samps.size(); ch++)
            {
                ipp32fc_signal[ch].resize(recvd_samps[ch].size());
                for (size_t s = 0; s < recvd_samps[ch].size() && s < samps_per_buff; s++)
                {
                    if constexpr (std::is_arithmetic<samp_type>::value)
                    {
                        short realPart = recvd_samps[ch][s];

                        ipp32fc_signal[ch][s].re = static_cast<float>(realPart);  
                        ipp32fc_signal[ch][s].im = 0.0f;
                    } else 
                    {
                        short realPart = recvd_samps[ch][s].real();
                        short imagPart = recvd_samps[ch][s].imag();

                        ipp32fc_signal[ch][s].re = static_cast<float>(realPart); 
                        ipp32fc_signal[ch][s].im = static_cast<float>(imagPart);  
                    }
                    
                    ipp32fc_signal[ch][s].re /= 32767.0;
                    ipp32fc_signal[ch][s].im /= 32767.0;
                }
            }
        }

        std::vector<double> compute_FFT(const std::vector<std::vector<Ipp32fc>> samps)
        {
            Ipp32fc FftDst[fftSize];
            memset(FftDst, 0, sizeof(FftDst));
            Ipp32f pDstIm[fftSize];
            memset(pDstIm, 0, sizeof(pDstIm));
            Ipp32s pSortIndx[fftSize];
            memset(pSortIndx, 0, sizeof(pSortIndx));
            Ipp32fc pWinDst[sigSize];
            memset(pWinDst, 0, sizeof(pWinDst));
            
            // Get sizes for required buffers
            ippsFFTGetSize_C_32fc(FFTOrder, flag, ippAlgHintNone, &sizeSpec, &sizeInit, &sizeBuffer);
            
            // Allocate memory for required buffers
            Ipp8u* pMemSpec_local = (Ipp8u*) ippMalloc(sizeSpec);
            Ipp8u* pMemInit_local = nullptr;
            Ipp8u* pMemBuffer_local = nullptr;

            if (sizeInit > 0)
            {
                pMemInit_local = (Ipp8u*) ippMalloc(sizeInit);
            }
            
            if (sizeBuffer > 0)
            {
                pMemBuffer_local = (Ipp8u*) ippMalloc(sizeBuffer);
            }

            IppsFFTSpec_C_32fc *pSpec_local = nullptr;

            // Initialize FFT specification structure
            ippsFFTInit_C_32fc(&pSpec_local, FFTOrder, flag, ippAlgHintNone, pMemSpec_local, pMemInit_local);

            // Free initialization buffer
            if (pMemInit_local) 
            {
                ippFree(pMemInit_local);
            }

            // Allocate sorting buffer
            int sort_buffer_size_local;
            ippsSortRadixIndexGetBufferSize(fftSize, ipp32f, &sort_buffer_size_local); 

            Ipp8u* pSortMemSpec_local = (Ipp8u*) ippMalloc(sort_buffer_size_local);
            if (!pSortMemSpec_local)
            {
                std::cerr << "[IppFFT] Memory allocation failed for sorting buffer!" << std::endl;
                if (pMemBuffer_local) ippFree(pMemBuffer_local);
                if (pMemSpec_local) ippFree(pMemSpec_local);
                return std::vector<double>();
            }
            
            // Perform WOLA
            Ipp32fc pWolaDst[sigSize/WOLA_ORDER];
            for (size_t ch = 0; ch < samps.size(); ch++)
            {
                // Multiply by Hamming window
                int curr_sigSize = samps[ch].size();
                ippsWinHamming_32fc(samps[ch].data(), pWinDst, curr_sigSize);

                size_t wolaIdx = 0;
                for (size_t col = 0; col < curr_sigSize; col += WOLA_ORDER)
                {
                    Ipp32fc tmp = {0, 0};  // CHANGED: Local variable instead of member
                    for (size_t w = col; w < col + WOLA_ORDER; w++)
                    {
                        tmp.re += pWinDst[w].re;
                        tmp.im += pWinDst[w].im;
                    }
                    tmp.re /= WOLA_ORDER;
                    tmp.im /= WOLA_ORDER;

                    pWolaDst[wolaIdx] = tmp;
                    wolaIdx++;   
                }
          
                // Perform FFT on WOLA's output
                ippsFFTFwd_CToC_32fc(pWolaDst, FftDst, pSpec_local, pMemBuffer_local);
                
                for(int i = 0; i < fftSize; i++)
                {
                    if(std::isnan(FftDst[i].im) || std::isnan(FftDst[i].re))  
                    {
                        if (pMemBuffer_local) ippFree(pMemBuffer_local);
                        if (pMemSpec_local) ippFree(pMemSpec_local); 
                        if (pSortMemSpec_local) ippFree(pSortMemSpec_local);

                        return std::vector<double>();
                    }
                }
          
                // Calculate FFT magnitude
                status = ippsMagnitude_32fc(FftDst, pDstIm, fftSize);
                
                // Sort radixes by highest amplitude
                sortStatus = ippsSortRadixIndexDescend_32f(pDstIm, sizeof(Ipp32f), pSortIndx, fftSize, pSortMemSpec_local);
                if (sortStatus != ippStsNoErr) {
                    std::cerr << "[IppFFT] Sorting failed with error code: " << sortStatus << std::endl;
                }

                max_amp = 20 * log10(pDstIm[pSortIndx[0]]);

                // Find peaks above threshold
                for (int i = 0; i < fftSize; i++)
                {
                    double amp = pDstIm[pSortIndx[i]];  
                    amp = 20 * log10(amp);
                    if (amp < amp_th)
                        break;

                    if(amp > max_amp - 3) 
                    {
                        Ipp32s current = pSortIndx[i] < fftSize/2 ? pSortIndx[i] : pSortIndx[i] - fftSize;  
                        double freq = fs / WOLA_ORDER * current / fftSize;  

                        bool is_append = true;  
                        for (size_t findex = 0; findex < freqs.size(); findex++)
                        {
                            double lastFreq = freqs.at(findex); 
                            if(std::abs(lastFreq - freq) < min_bw) 
                            {
                                is_append = false;                    
                                break;
                            }
                        }
                        if (i == 0 || is_append)
                        {
                            freqs.push_back(freq);
                        }
                    }
                }
            }

            // Free buffers
            if (pMemBuffer_local) ippFree(pMemBuffer_local);
            if (pMemSpec_local) ippFree(pMemSpec_local); 
            if (pSortMemSpec_local) ippFree(pSortMemSpec_local);

            return freqs;
        }

        // Member variables
        size_t max_rx_channels;
        size_t samps_per_buff;
        double fs;

        // IPP variables
        IppStatus sortStatus;
        std::vector<std::vector<Ipp32fc>> ippSrc;

        IppStatus status;
        int sigSize;

        int fftSize; 
        int FFTOrder;
            
        int sizeSpec = 0;
        int sizeInit = 0;
        int sizeBuffer = 0;
        int flag = IPP_FFT_NODIV_BY_ANY;

        double max_amp = 0;
        double amp_th = -15;

        double min_bw = 6.25e3;
};

template <typename samp_type>
class SignalTnsmt
{
    public:
        SignalTnsmt(uhd::usrp::multi_usrp::sptr usrp,
            uhd::tx_streamer::sptr tx_stream,
            const std::string& file,
            size_t samps_per_buff,
            size_t max_tx_channels,
            const std::vector<size_t>& tx_channels)
                  : usrp(usrp),
                    tx_stream(tx_stream),
                    file(file),
                    samps_per_buff(samps_per_buff),
                    max_tx_channels(max_tx_channels),
                    tx_channels(tx_channels),
                    SigGen(usrp, file, samps_per_buff, tx_channels),
                    usrpConfig(usrp)
        {}

        void send_from_file() 
        {   
            while (!stop_signal_called)
            {
                std::vector<samp_type> jam_buff(samps_per_buff);
                std::vector<samp_type*> jam_buffer(tx_stream->get_num_channels());
                
                for (size_t ch = 0; ch < tx_stream->get_num_channels(); ch++)
                {
                    jam_buffer[ch] = &jam_buff.front() + (ch * samps_per_buff);
                }

                {
                    std::unique_lock<std::mutex> lock(signal_mutex);
                    RX_done_cv.wait(lock, [] { return signal_received.load(); });
                }
                std::cout << "[TX] Starting transmission..." << std::endl;

                const double tx_rate = usrp->get_tx_rate();
                
                if (tx_from_file)
                {
                                        
                    std::vector<double> usrp_tx_freqs(tx_channels.size());
                    for (size_t i = 0; i < tx_channels.size(); i++)
                    {
                        usrp_tx_freqs[i] = usrp->get_tx_freq(tx_channels[i]);
                        std::ifstream infile(file.c_str(), std::ifstream::binary);
            
                        for (size_t f = 0; f < freqs.size(); f++)
                        {
                            std::cout << "[TX] Transmitting for " 
                                      << freqs[f] / 1e6 << "..." << std::endl;
                            if (freqs[f] != usrp_tx_freqs[i])
                            {
                                usrpConfig.set_channel_freq(freqs[f], 0, false, false, tx_channels);
                                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                            }
                            infile.clear();
                            infile.seekg(0, std::ios::beg);

                            size_t sample_idx = 0;
                            
                            size_t samples_sent = 0;
                            const size_t total_samps = static_cast<size_t>(tx_rate * JAM_DUR);
                            while (samples_sent < total_samps)
                            {
                                size_t samples_remaining = total_samps - samples_sent;
                                size_t samples_to_read = std::min(jam_buff.size(), samples_remaining);

                                infile.read((char*)&jam_buff.front(), samples_to_read * sizeof(samp_type));
                                size_t num_tx_samps = size_t(infile.gcount() / sizeof(samp_type));
                                
                                if (num_tx_samps == 0)
                                {
                                    infile.clear();
                                    infile.seekg(0, std::ios::beg);
                                    continue;
                                }

                                samplesTX(jam_buffer, num_tx_samps);
                                samples_sent += num_tx_samps;
                            }

                            /*
                            size_t samples_sent = 0;
                            const size_t total_samps = static_cast<size_t>(tx_rate * JAM_DUR);

                            while (!infile.eof() || samples_sent < total_samps)
                            {
                                infile.read((char*)&jam_buff.front(), jam_buff.size() * sizeof(samp_type));
                                size_t num_tx_samps = size_t(infile.gcount() / sizeof(samp_type));
                                samplesTX(jam_buffer, num_tx_samps);
                                samples_sent += num_tx_samps;
                            }
                            */
                            std::cout << "[TX] Done transmitting..." << std::endl;
                        }
                        infile.close();
                    }

                    /*
                    std::ifstream infile(file.c_str(), std::ifstream::binary)
            
                    for (size_t f = 0; f < freqs.size(); f++)
                    {
                        std::cout << "[TX] Transmitting for " << freqs[f] / 1e6 << "..." << std::endl;

                        infile.clear();
                        infile.seekg(0, std::ios::beg);

                        size_t sample_idx = 0;

                        while (!infile.eof())
                        {
                            infile.read((char*)&jam_buff.front(), jam_buff.size() * sizeof(samp_type));
                            size_t num_tx_samps = size_t(infile.gcount() / sizeof(samp_type));

                            if (num_tx_samps > 0) 
                            {
                                const double freq_base = 2.0 * M_PI * freqs[f] / tx_rate;

                                if constexpr (std::is_same_v<samp_type, std::complex<short>> ||
                                            std::is_same_v<samp_type, std::complex<int16_t>>)
                                {   
                                    for (size_t i = 0; i < num_tx_samps; i++)
                                    {
                                        double phi = freq_base * (sample_idx + i);
                                        std::complex<double> shift = std::exp(std::complex<double>(0, phi));

                                        std::complex<double> sample(jam_buff[i].real(), jam_buff[i].imag());
                                        sample *= shift;
                                        jam_buff[i] = samp_type(static_cast<short>(sample.real()),
                                                                static_cast<short>(sample.imag()));
                                    }
                                    sample_idx += num_tx_samps;
                                }

                                else if constexpr (std::is_same_v<samp_type, double> ||
                                                std::is_same_v<samp_type, float>)
                                {                                   
                                    for (size_t i = 0; i < num_tx_samps/2; i++)
                                    {
                                        double phi = freq_base * (sample_idx + i);
                                        std::complex<double> shift = std::exp(std::complex<double>(0, phi));

                                        std::complex<double> sample(jam_buff[2 * i], jam_buff[2 * i + 1]);
                                        sample *= shift;
                                        jam_buff[2 * i] = sample.real() ;
                                        jam_buff[2 * i + 1] = sample.imag();
                                    }
                                    sample_idx += num_tx_samps/2;
                                }

                                samplesTX(jam_buffer, num_tx_samps);
                            }
                        }
                        std::cout << "[TX] Done transmitting..." << std::endl;
                    }
                    infile.close();
                    */
                }
                
                else
                { 
                    std::vector<std::complex<short>> signal = applyJamming(Technique::CW);
                    
                    const size_t total_samps = static_cast<size_t>(tx_rate * JAM_DUR);
                    size_t samples_sent = 0;

                    uhd::tx_metadata_t md;
                    md.start_of_burst = true;
                    md.end_of_burst   = false;
                    md.has_time_spec  = false;

                    while (samples_sent < total_samps)
                    {
                        size_t rem_samps = total_samps - samples_sent;
                        size_t num_samps = std::min(rem_samps, signal.size()); 

                        if (samples_sent + num_samps >= total_samps)
                        {
                            md.end_of_burst = true;
                        }

                        size_t sent = tx_stream->send(signal.data(), num_samps, md, 1.0);

                        if (sent != num_samps)
                        {
                            std::cerr << "[TX] Underrun or timeout. Sent " << sent
                                << " of " << num_samps << " samples." << std::endl;
                        }

                        md.start_of_burst = false;
                        samples_sent += sent;

                    }

                    md.end_of_burst = true;
                }
                
                {
                    std::lock_guard<std::mutex> lock(signal_mutex);
                    signal_received.store(false);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
                TX_done_cv.notify_one();
            }
        }

        void samplesTX(const std::vector<samp_type*>& buffs, size_t num_tx_samps)
        {
            uhd::tx_metadata_t md;
            md.start_of_burst = false;
            md.end_of_burst   = false;
            const size_t samples_sent = tx_stream->send(buffs, num_tx_samps, md);
            if (samples_sent != num_tx_samps) {
                UHD_LOG_ERROR("TX-STREAM",
                    "The tx_stream timed out sending " << num_tx_samps << " samples ("
                                                    << samples_sent << " sent).");
                return;
            }
        }

    private:
        std::vector<std::complex<short>> applyJamming(Technique mode)
        {
            switch (mode)
            {
                case Technique::CW:
                    return SigGen.create_cw();
                
                case Technique::Multi:
                    return {};

                case Technique::Chirp:
                    return {};
            }
            return {};
        }

        uhd::usrp::multi_usrp::sptr usrp;
        uhd::tx_streamer::sptr tx_stream;
        std::vector<size_t> tx_channels;
        size_t max_tx_channels;
        size_t samps_per_buff;
        std::string file;

        SignalGenerator SigGen;
        USRPconfig usrpConfig;
};

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // variables to be set by po
    std::string args, type, file, ant, subdev, ref, wirefmt, rx_channels, tx_channels;
    size_t total_num_samps, spb;
    double rate, freq, gain, bw, setup_time, lo_offset, delay;

    std::vector<std::thread> rx_threads; 
    std::vector<size_t> rx_channel_list;
    std::vector<size_t> tx_channel_list;
    std::vector<std::string> rx_channel_strings;
    std::vector<std::string> tx_channel_strings;
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("type", po::value<std::string>(&type)->default_value("short"), "sample type: double, float, or short")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("spb", po::value<size_t>(&spb)->default_value(8192), "samples per buffer")
        ("rate", po::value<double>(&rate)->default_value(50e6), "rate of incoming samples")
        ("freq", po::value<double>(&freq)->default_value(146e6), "RF center frequency in Hz")
        ("lo-offset", po::value<double>(&lo_offset)->default_value(0.0),
            "Offset for frontend LO in Hz (optional)")
        ("gain", po::value<double>(&gain)->default_value(0.0), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("subdev", po::value<std::string>(&subdev), "subdevice specification")
        ("rx_channels", po::value<std::string>(&rx_channels)->default_value("0"), "which channel(s) to use for RX (specify \"0\", \"1\", \"0,1\", etc)")
        ("tx_channels", po::value<std::string>(&tx_channels)->default_value("1"), "which channel(s) to use for TX (specify \"0\", \"1\", \"0,1\", etc)")
        ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref), "reference source (internal, external, mimo)")
        ("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8, sc16 or s16)")
        ("setup", po::value<double>(&setup_time)->default_value(1.0), "seconds of setup time")
        ("progress", "periodically display short-term bandwidth")
        ("stats", "show average bandwidth on exit")
        ("sizemap", "track packet size and display breakdown on exit. Use with multi_streamer option if CPU limits stream rate.")
        ("null", "run without writing to file")
        ("continue", "don't abort on a bad packet")
        ("skip-lo", "skip checking LO lock status")
        ("int-n", "tune USRP with integer-N tuning")
        ("delay", po::value<double>(&delay)->default_value(0.0), "specify a delay between repeated transmission of file (in seconds)")
        ("file", po::value<std::string>(&file)->default_value("home/user/Projects/files_for_phi/tst/cw.sc16"), "name of the file to read binary samples from")
        ("multi_streamer", "Create a separate streamer per channel.")
        ("repeat", "Repeats.")
        ("channel", "channel.")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
   
    // print the help message
    if (vm.count("help"))
    {
        std::cout << std::endl
                  << "This application receives samples on RX channel of the USRP device.\n"
                  << std::endl;
        return ~0;
    } 

    /*
    if (vm.count("file"))
    {
        tx_from_file = true;
    }
    */
    file = "/home/user/Projects/files_for_phi/tst/cw_FM.sc16";

    bool bw_summary             = vm.count("progress") > 0;
    bool stats                  = vm.count("stats") > 0;
    bool null                   = vm.count("null") > 0;
    bool enable_size_map        = vm.count("sizemap") > 0;
    bool continue_on_bad_packet = vm.count("continue") > 0;
    bool multithread            = vm.count("multi_streamer") > 0;
    bool repeat                 = true;

    if (enable_size_map)
        std::cout << "Packet size tracking enabled - will only recv one packet at a time!"
                  << std::endl;

    // create a usrp device
    std::cout << std::endl;
    std::cout << "Creating the usrp device with: " << args << "..." << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    // create a class alias
    USRPconfig config(usrp);

    // Parse channels
    config.parse_channels(rx_channel_strings, rx_channels, usrp->get_rx_num_channels(), rx_channel_list);
    config.parse_channels(tx_channel_strings, tx_channels, usrp->get_tx_num_channels(), tx_channel_list);

    // Lock mboard clocks
    if (vm.count("ref")) {
        usrp->set_clock_source(ref);
    }

    if (vm.count("subdev")) {
        usrp->set_rx_subdev_spec(subdev);
        usrp->set_tx_subdev_spec(subdev);
    }
    
    std::cout << "Using Device: " << usrp->get_pp_string() << std::endl;

    // Validate the sample rate
    if (rate <= 0.0) {
        std::cerr << "Please specify a valid sample rate" << std::endl;
        return ~0;
    }

    // Set the sample rate
    /*if (!vm.count("rate"))*/ 
    rate = 50e6;
    config.set_channel_rate(rate, true, rx_channel_list);
    config.set_channel_rate(rate, false, tx_channel_list);

    // Set the center frequency
    /*if (!vm.count("freq"))*/
    freq = 140e6;
    config.set_channel_freq(freq, lo_offset, true, vm.count("int-n"), rx_channel_list);
    config.set_channel_freq(freq, lo_offset, false, vm.count("int-n"), tx_channel_list);

    // Set the channels gain
    if (!vm.count("gain")) gain = 10;
    if (vm.count("gain")) {
        config.set_channel_gain(gain, true, rx_channel_list);
        config.set_channel_gain(gain, false, tx_channel_list);
    }

     // Set the IF filter bandwidth
    if (vm.count("bw")) {
        config.set_channel_bw(bw, true, rx_channel_list);
        config.set_channel_bw(bw, false, tx_channel_list);
    }

    // Set the antenna
    if (vm.count("ant")) {
        config.set_channel_antenna(ant, true, rx_channel_list);
        config.set_channel_antenna(ant, false, tx_channel_list);
    }

    // Allow time for tuning
    std::this_thread::sleep_for(1s * setup_time);

    // Check Ref and LO Lock detect
    if (not vm.count("skip-lo")) {
        // LO lock function for RX and TX
        config.lock_lo(true, rx_channel_list, setup_time);
        config.lock_lo(false, tx_channel_list, setup_time);

        if (ref == "mimo") {
            config.check_locked_sensor(
                usrp->get_mboard_sensor_names(0),
                "mimo_locked",
                [usrp](const std::string& sensor_name) {
                    return usrp->get_mboard_sensor(sensor_name);
                },
                setup_time);
        } 
        if (ref == "external") {
            config.check_locked_sensor(
                usrp->get_mboard_sensor_names(0),
                "ref_locked",
                [usrp](const std::string& sensor_name) {
                    return usrp->get_mboard_sensor(sensor_name);
                },
                setup_time);
        }
    }
    
    if (total_num_samps == 0) {
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }   
    
    std::vector<size_t> rx_chans_in_thread;
    std::vector<double> rx_rates(rx_channel_list.size());

    std::string cpu_format;
    if (type == "double")
        cpu_format = "fc64";
    else if (type == "float")
        cpu_format = "fc32";
    else if (type == "short")
        cpu_format = "sc16";

    // Create RX streamer
    uhd::stream_args_t rx_stream_args(cpu_format, wirefmt);
    rx_stream_args.channels             = rx_channel_list; 
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(rx_stream_args);

    // Create TX streamer
    uhd::stream_args_t tx_stream_args(cpu_format, wirefmt);
    tx_stream_args.channels             = tx_channel_list; 
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(tx_stream_args);

#define recv_samples_args                                                            \
    usrp,                                                                            \
    rx_stream,                                                                       \
    rx_chans_in_thread,                                                              \
    rx_channel_list.size(),                                                          \
    spb,                                                                             \
    total_num_samps,                                                                 \
    std::ref(rx_rates[i]),                                                           \
    stats,                                                                           \
    null,                                                                            \
    enable_size_map,                                                                 \
    continue_on_bad_packet,                                                          \
    th_prefix

    auto create_receiver = [&](size_t i, const std::string& th_prefix,
                               const std::vector<size_t>& rx_chans)
    {
        if (wirefmt == "s16") {
            if (type == "double") {
                auto receiver = std::make_shared<SignalRecv<double>>(usrp->get_rx_num_channels(), spb, rate);
                receiver->samplesRX(recv_samples_args);
            } else if (type == "float") {
                auto receiver = std::make_shared<SignalRecv<float>>(usrp->get_rx_num_channels(), spb, rate);
                receiver->samplesRX(recv_samples_args);
            } else if (type == "short") {
                auto receiver = std::make_shared<SignalRecv<short>>(usrp->get_rx_num_channels(), spb, rate);
                receiver->samplesRX(recv_samples_args);
            } else
                throw std::runtime_error("Unknown type " + type);
        } else {
            if (type == "double") {
                auto receiver = std::make_shared<SignalRecv<std::complex<double>>>(usrp->get_rx_num_channels(), spb, rate);
                receiver->samplesRX(recv_samples_args);
            } else if (type == "float") {
                auto receiver = std::make_shared<SignalRecv<std::complex<float>>>(usrp->get_rx_num_channels(), spb, rate);
                receiver->samplesRX(recv_samples_args);
            } else if (type == "short") {
                auto receiver = std::make_shared<SignalRecv<std::complex<short>>>(usrp->get_rx_num_channels(), spb, rate);
                receiver->samplesRX(recv_samples_args);
            } else
                throw std::runtime_error("Unknown type " + type);
        }
    };

    for (size_t i = 0; i < rx_channel_list.size(); i++) {
        std::string th_prefix = "";
        if (multithread) {
            rx_chans_in_thread.clear();
            rx_chans_in_thread.push_back(rx_channel_list[i]);
            th_prefix = "Thread " + std::to_string(i) + ":\n";
        } else {
            rx_chans_in_thread = rx_channel_list;
        }

        rx_threads.push_back(std::thread([&, i, th_prefix]() {
            create_receiver(i, th_prefix, rx_chans_in_thread);
        }));

        if (!multithread) {
            break;
        } 
    }

#undef recv_samples_args

    auto create_transmitter = [&]()
    {
        if (type == "double"){
            auto transmitter = std::make_shared<SignalTnsmt<double>>(usrp, 
                                                                    tx_stream, 
                                                                    file, 
                                                                    spb, 
                                                                    usrp->get_tx_num_channels(), 
                                                                    tx_channel_list);
            transmitter->send_from_file();
        } else if (type == "float") {
            auto transmitter = std::make_shared<SignalTnsmt<float>>(usrp, 
                                                                    tx_stream, 
                                                                    file, 
                                                                    spb, 
                                                                    usrp->get_tx_num_channels(), 
                                                                    tx_channel_list);
            transmitter->send_from_file();
        } else if (type == "short") {
            auto transmitter = std::make_shared<SignalTnsmt<short>>(usrp, 
                                                                    tx_stream, 
                                                                    file, 
                                                                    spb, 
                                                                    usrp->get_tx_num_channels(), 
                                                                    tx_channel_list);
            transmitter->send_from_file();
        } else
            throw std::runtime_error("Unknown type " + type);
    };

    std::thread TXthread(create_transmitter);

    if (repeat and delay > 0.0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(int64_t(delay * 1000)));
    }
    
    if (TXthread.joinable())
    {
        TXthread.join();
    }
    
    double total_time = 0.0;
    if (total_num_samps > 0)
    {
        total_time = std::ceil(total_num_samps / usrp->get_rx_rate());
    }

    // Wait a bit extra for the first updates from each thread
    std::this_thread::sleep_for(500ms);

    const auto end_time = std::chrono::steady_clock::now() + (total_time - 1) * 1s;

    while (rx_threads.size() > 0 
           && (std::chrono::steady_clock::now() < end_time || total_time == 0)
           && !stop_signal_called)
    {
        std::this_thread::sleep_for(1s);

        // Remove any threads that are finished
        for (size_t i = 0; i < rx_threads.size(); i++)
        {
            if (!rx_threads[i].joinable())
            {
                rx_threads.erase(rx_threads.begin() + i);
                rx_rates[i] = 0;
            }
        }
        
        // Report the bandwidth of remaining threads
        if (bw_summary && rx_threads.size() > 0)
        {
            const std::lock_guard<std::mutex> lock(recv_mutex);
            std::cout << "\t"
                      << (std::accumulate(std::begin(rx_rates), std::end(rx_rates), 0) / 1e6
                             / rx_threads.size())
                      << " Msps" << std::endl;
        }
    }

    // join any remaining threads
    for (size_t i = 0; i < rx_threads.size(); i++)
    {
        if (rx_threads[i].joinable())
        {
            rx_threads[i].join();
        }
    }

    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}


