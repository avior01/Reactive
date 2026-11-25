#pragma once

#include "reactive/Common.hpp"

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
