#pragma once

#include "reactive/Common.hpp"

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
