#pragma once

#include "reactive/Common.hpp"
#include "reactive/USRPConfig.hpp"
#include "reactive/SignalGenerator.hpp"

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
