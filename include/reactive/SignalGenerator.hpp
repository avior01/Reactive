#pragma once

#include "reactive/Common.hpp"

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
                    if (std::abs(freqs[i] - freqs[j]) <= cfg::kToneSpacingHz)
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
            if (cfg::kNoiseBwHz > 0)
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
            const double scale = cfg::kFullScale;

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

            std::mt19937 gen(cfg::kNoiseSeed);
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
            const int filter_order = cfg::kNoiseFilterOrder;
            double rFreq = cfg::kNoiseBwHz / usrp->get_tx_rate(tx_channels[0]);
            Ipp64f taps[2 * (filter_order + 1)];

            std::cout << "Filter: Noise BW = " << cfg::kNoiseBwHz/1e6 << " MHz," 
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
