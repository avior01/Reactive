# Reactive

A **reactive jammer** built on a USRP software-defined radio (SDR). It continuously
listens on a receive (RX) channel, detects signals present in the spectrum, and — the
moment a signal appears — retunes the transmit (TX) channel to each detected frequency
and emits a jamming waveform. When transmission finishes it returns to listening.

The name refers to this **listen → detect → react (jam)** loop, as opposed to a *barrage*
jammer that transmits blindly all the time.

> Built from scratch as an independent project: a real-time SDR/DSP engine in C++ plus a
> Python waveform-generation toolkit. For the full design write-up, see
> [`docs/DOCUMENTATION.md`](docs/DOCUMENTATION.md).

---

## How it works

```
   RX channel ──► SignalRecv<T> ──► detect ──► freqs[] ──► SignalTnsmt<T> ──► TX channel
   (capture spb   WOLA → FFT →      (peaks)    (shared)    retune to each     (jam for
    samples)      peak picking                              detected freq      JAM_DUR s)
                        ▲                                          │
                        └──────────── RX/TX handshake ◄────────────┘
                              (atomics + condition variables)
```

- **Detection pipeline:** Hamming window → WOLA decimation → forward FFT (Intel IPP) →
  magnitude → radix-sort peak picking with a 3 dB window and frequency de-duplication.
- **Concurrency:** dedicated RX and TX threads synchronized by a two-phase
  condition-variable rendezvous (`std::atomic`, `std::mutex`, `std::condition_variable`).
- **Two jam sources:** transmit a pre-generated IQ file, or synthesize a CW/noise waveform
  in C++ at runtime.

## Repository layout

```
include/reactive/   C++ engine, split by component (header-per-class; one translation unit)
  Common.hpp          shared includes, macros, global sync state, Technique enum
  USRPConfig.hpp      USRP device configuration
  SignalGenerator.hpp in-C++ CW/noise waveform synthesis
  SignalRecv.hpp      RX capture + detection (template on sample type)
  SignalTnsmt.hpp     TX jamming loop (template on sample type)
src/main.cpp        entry point: CLI, device/stream setup, RX/TX threads
python/jams/        waveform-generation package (CW, FM, chirp, barrage, DTMF, PSK, …)
python/j2s_sweep.py Jam-to-Signal-ratio sweep (legacy tooling)
docs/               full documentation
```

## Build

Requires a C++20 compiler, [UHD](https://github.com/EttusResearch/uhd) ≥ 4.1.0,
Intel oneAPI IPP, and Boost ≥ 1.65 (see `CMakeLists.txt`).

```bash
mkdir -p build && cd build
cmake ..
make            # produces the `reactive` executable
```

## Run

```bash
./reactive --rate 50e6 --freq 140e6 --rx_channels 0 --tx_channels 1 \
           --gain 10 --file /path/to/jam.sc16
# Press Ctrl+C to stop.
```

See [`docs/DOCUMENTATION.md`](docs/DOCUMENTATION.md) §6 for the full command-line reference.

## Generating jam waveforms (Python)

The `jams` package produces the raw interleaved-int16 (`sc16`) IQ files the engine
transmits:

```bash
cd python
python -m jams          # builds the default cw_FM.sc16 and plots it
```

```python
from jams import jam, files

j = jam(fs=50e6, jam_duration=0.5)
signal = j.create_chirp(f_start=-5e6, f_end=5e6, chirp_duration=0.1, noise_bw=6.25e3)
files(None, "out_dir").save_to_sc16(signal, "chirp")
```

Available generators include CW / multi-CW, FM, linear chirp, cyclic tones, barrage,
DTMF, and BPSK/QPSK — each optionally shaped by band-limited Gaussian noise.

## Tech stack

**C++20** · **UHD** (USRP driver) · **Intel IPP** (FFT, windowing, IIR, radix sort) ·
**Boost** (program_options, format) · **Python** (NumPy, SciPy, Matplotlib)
