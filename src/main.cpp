#include "reactive/Common.hpp"
#include "reactive/USRPConfig.hpp"
#include "reactive/SignalGenerator.hpp"
#include "reactive/SignalRecv.hpp"
#include "reactive/SignalTnsmt.hpp"

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


