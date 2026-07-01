# Reactive — Documentation

## 1. Overview

**Reactive** is a *reactive jammer* built on top of a USRP software-defined radio
(SDR). It continuously listens on a receive (RX) channel, detects the presence of
signals in the spectrum, and — the moment a signal is detected — transmits a
jamming waveform on the transmit (TX) channel tuned to each detected frequency.
Once transmission finishes it returns to listening. The name "reactive" refers to
this listen → detect → react (jam) loop, as opposed to a "barrage" jammer that
transmits blindly all the time.

The project has two parts:

| Part | Language | Role |
|------|----------|------|
| `reactive.cpp` | C++20 | The real-time engine that runs on the radio: RX capture, signal detection, and TX jamming. |
| `PyFiles/` | Python | Offline tooling that *generates the binary IQ sample files* the C++ engine transmits (and helper analysis/plotting). |

Hardware / library stack:

- **UHD** (USRP Hardware Driver) ≥ 4.1.0 — controls the Ettus/NI USRP radio.
- **Intel IPP** (Integrated Performance Primitives) — FFT, windowing, IIR filtering,
  radix sort, complex-number types used in the DSP hot paths.
- **Boost** — `program_options` (CLI), `format`, `algorithm/string`, and (on Linux)
  `filesystem`, `process`, `random`.
- **C++ standard library threading** — `std::thread`, `std::mutex`,
  `std::condition_variable`, `std::atomic` for the RX/TX handshake.

---

## 2. Directory layout

```
Reactive/
├── CMakeLists.txt            # Build configuration (UHD + IPP + Boost + Python/pybind11)
├── reactive.cpp              # The entire C++ application (single translation unit)
├── PyFiles/
│   ├── jams.py               # Jamming-waveform generator + file I/O + plotting  ← RELEVANT
│   └── J2S_file_creation.py  # Jam-to-Signal-ratio file sweep                     ← NOT relevant to current project
└── build/                    # CMake out-of-source build tree (generated artifacts)
    ├── CMakeCache.txt
    ├── cmake_install.cmake
    ├── compile_commands.json
    ├── Makefile
    └── CMakeFiles/
```

> **Note on the Python files (per project owner):**
> `jams.py` is the relevant generator — it produces the binary IQ files that
> `reactive.cpp` reads and transmits. `J2S_file_creation.py` builds a *Jam-to-Signal
> ratio* sweep (mixing a jammer with a recorded signal at varying power ratios) and
> **is not part of the current Reactive project** — it is legacy/experimental tooling.

---

## 3. How the system works (end-to-end)

```
        ┌───────────────────────────────────────────────────────────────┐
        │                         USRP radio                            │
        │   RX channel(s)                          TX channel(s)        │
        └──────┬────────────────────────────────────────────┬──────────┘
               │ IQ samples                                  │ IQ samples
               ▼                                             ▲
    ┌─────────────────────┐                       ┌─────────────────────┐
    │  RX thread(s)       │                       │  TX thread          │
    │  SignalRecv<T>      │                       │  SignalTnsmt<T>     │
    │                     │  signal_received=true │                     │
    │  capture spb samps  │ ────────────────────► │  waits on RX_done_cv│
    │  detect_signal():   │   freqs[] (global)    │  reads file / gen   │
    │   WOLA → FFT → peak  │                       │  retunes TX to each │
    │   pick → freqs[]    │ ◄──────────────────── │  detected freq, TX  │
    │  restart on TX done │   signal_received=... │  for JAM_DUR sec    │
    └─────────────────────┘   TX_done_cv          └─────────────────────┘
```

### The reactive loop, step by step

1. **Setup (`UHD_SAFE_MAIN`)** — parse CLI options, create the USRP device, configure
   RX/TX rate, frequency, gain, (optional) bandwidth/antenna, lock the LOs, and build
   the RX and TX streamers.
2. **RX thread(s)** run `SignalRecv::samplesRX`, streaming `spb` samples per iteration.
3. Once a full buffer (`spb` samples) is collected, `detect_signal()` runs the DSP
   chain (WOLA decimation → FFT → magnitude → peak picking) and returns the list of
   detected baseband frequencies.
4. Detected frequencies are converted to absolute RF frequencies
   (`usrp->get_rx_freq() + offset`) and stored in the **global `freqs` vector**. The
   RX thread sets `signal_received = true` and notifies `RX_done_cv`.
5. **TX thread** (`SignalTnsmt::send_from_file`) was blocked on `RX_done_cv`. It wakes,
   and for each detected frequency it retunes the TX LO to that frequency and transmits
   the jamming waveform for `JAM_DUR` seconds.
6. When TX finishes, it sets `signal_received = false`, sleeps briefly, and notifies
   `TX_done_cv`.
7. The RX thread — which was waiting on `TX_done_cv` for `signal_received` to go false —
   wakes, clears `freqs`, and restarts listening. Loop repeats until `Ctrl+C`.

The whole thing stops when `SIGINT` sets the atomic `stop_signal_called` (installed via
`sig_int_handler`).

---

## 4. `reactive.cpp` in detail

### 4.1 Compile-time configuration (macros)

| Macro | Value | Meaning |
|-------|-------|---------|
| `WOLA_ORDER` | `4` | Weighted-overlap-add decimation factor. The FFT runs on `spb / WOLA_ORDER` points, trading frequency resolution for a cleaner, decimated spectrum and lower FFT cost. |
| `JAM_DUR` | `10` | Duration, in seconds, to transmit the jam per detected frequency. |
| `NOISE_BW` | `100e3` | Bandwidth (Hz) of the Gaussian noise multiplied onto the CW jam (used by the on-the-fly `create_cw` path). |

### 4.2 Global synchronization state

- `std::atomic<bool> stop_signal_called` — set by `Ctrl+C`; ends all loops.
- `std::atomic<bool> signal_received` — the RX→TX handshake flag.
- `std::atomic<bool> tx_from_file{true}` — selects the TX source: read a pre-built IQ
  file (`true`) vs. synthesize the waveform in C++ at runtime (`false`).
- `std::mutex recv_mutex`, `signal_mutex` — protect console output / the handshake.
- `std::condition_variable RX_done_cv`, `TX_done_cv` — the two-phase RX↔TX rendezvous.
- `std::vector<double> freqs` — **global** list of detected RF frequencies, shared
  between RX (writer) and TX (reader).

### 4.3 `enum class Technique { CW, Multi, Chirp }`

Selects the runtime-synthesis jamming technique. Only `CW` is implemented in
`applyJamming`; `Multi` and `Chirp` are stubs returning `{}`.

### 4.4 `class USRPconfig`

Thin wrapper around `uhd::usrp::multi_usrp` that centralizes device configuration.

| Method | Purpose |
|--------|---------|
| `check_locked_sensor(...)` | Poll a sensor (e.g. `lo_locked`, `ref_locked`, `mimo_locked`) until it reports locked or times out. |
| `parse_channels(...)` | Split a CLI channel string (e.g. `"0,1"`) into a `vector<size_t>` and validate against the device's channel count. |
| `set_channel_rate(rate, is_rx, channels)` | Set and read back RX or TX sample rate. |
| `set_channel_freq(freq, lo_offset, is_rx, int_n, channels)` | Tune RX or TX LO(s), optionally in integer-N mode. |
| `set_channel_gain(...)` | Set/read RX or TX gain. |
| `set_channel_bw(...)` | Set/read analog front-end filter bandwidth. |
| `set_channel_antenna(...)` | Select antenna port. |
| `lock_lo(is_rx, channels, setup_time)` | Wait for LO lock on each channel via `check_locked_sensor`. |

### 4.5 `class SignalGenerator`

Synthesizes jamming waveforms **in C++ at runtime** (the alternative to reading a
pre-built file). Templated on nothing — it emits `std::complex<short>` (sc16).

| Method | Purpose |
|--------|---------|
| `create_cw()` | Build a multi-tone continuous-wave signal: for each detected frequency, add a complex exponential at the baseband offset `freqs[i] - tx_freq`. Tones closer than 6.25 kHz to an already-placed tone are skipped. Optionally multiplies in Gaussian noise (`NOISE_BW`), normalizes, and converts to sc16. |
| `normalize_signal(...)` | Scale so the max of \|I\| and \|Q\| equals 1.0. |
| `convert_to_short(...)` | Scale by 32767 and cast to `complex<short>` (sc16). |
| `create_gaussian_noise(N)` | Generate complex Gaussian noise (seeded `mt19937(42)`), normalize to unit RMS, low-pass filter it, renormalize. |
| `apply_lpf(noise, N)` | Butterworth low-pass (order 3) via IPP `ippsIIRGenLowpass_64f` + `ippsIIR_64f`, applied separately to I and Q. Cutoff = `NOISE_BW / tx_rate`. |

### 4.6 `template<typename samp_type> class SignalRecv`

The receiver + detector. `samp_type` is one of `short`, `float`, `double`, or their
`std::complex<>` versions (chosen at runtime from `--type` and `--wirefmt`).

Constructor precomputes DSP sizes:
- `sigSize = spb`
- `fftSize = spb / WOLA_ORDER`
- `FFTOrder = log2(spb / WOLA_ORDER)`

| Method | Purpose |
|--------|---------|
| `samplesRX(...)` | Main RX loop. Issues `STREAM_MODE_START_CONTINUOUS`, receives into per-channel buffers, handles timeout/overflow/error metadata, accumulates until `spb` samples are gathered, then calls `detect_signal`. On detection: converts baseband peaks to RF, sets `signal_received`, notifies TX, then blocks on `TX_done_cv` until TX completes before restarting. Also flushes the stream and can print stats / a packet-size map. |
| `detect_signal(recvd_samps)` | Converts samples to `Ipp32fc` and runs `compute_FFT`. |
| `adjust_samps_type(...)` | Convert incoming `samp_type` (real or complex) into normalized `Ipp32fc` (divide by 32767). Uses `if constexpr (std::is_arithmetic...)` to handle real vs. complex sample types at compile time. |
| `compute_FFT(samps)` | The detection core (see below). |

**`compute_FFT` — detection pipeline:**
1. Allocate IPP FFT spec/init/work buffers (`ippsFFTGetSize/Init_C_32fc`).
2. Apply a **Hamming window** (`ippsWinHamming_32fc`).
3. **WOLA decimation**: average every `WOLA_ORDER` consecutive samples into one, giving
   `sigSize/WOLA_ORDER` points.
4. **Forward FFT** (`ippsFFTFwd_CToC_32fc`); bail out if any NaN appears.
5. **Magnitude** (`ippsMagnitude_32fc`), then **radix sort descending**
   (`ippsSortRadixIndexDescend_32f`) to rank bins by amplitude.
6. **Peak picking**: `max_amp` = strongest bin (dB). Walk bins from strongest down;
   stop once amplitude drops below `amp_th` (−15 dB). Keep bins within 3 dB of `max_amp`.
   Convert bin index to frequency (`fs / WOLA_ORDER * bin / fftSize`, with negative-
   frequency wraparound). Deduplicate peaks closer than `min_bw` (6.25 kHz).

Key detection parameters (member vars): `amp_th = -15` dB, `min_bw = 6.25e3` Hz.

### 4.7 `template<typename samp_type> class SignalTnsmt`

The transmitter. Owns a `SignalGenerator` (`SigGen`) and a `USRPconfig` (`usrpConfig`).

| Method | Purpose |
|--------|---------|
| `send_from_file()` | Main TX loop. Blocks on `RX_done_cv` until `signal_received`. Then, **if `tx_from_file`**: opens the binary IQ file and, for each detected frequency, retunes the TX LO to that frequency, rewinds the file, and streams it in `spb`-sample chunks for `tx_rate * JAM_DUR` samples (looping the file if it's shorter). **Else** (`tx_from_file == false`): synthesizes a CW jam via `applyJamming(Technique::CW)` and bursts it for `JAM_DUR`. After transmitting, clears `signal_received`, sleeps 1 s, and notifies `TX_done_cv`. |
| `samplesTX(buffs, num_tx_samps)` | Low-level `tx_stream->send` wrapper with underrun logging. |
| `applyJamming(mode)` | Dispatch to a synthesis technique; only `CW` (`SigGen.create_cw()`) is implemented. |

> There are two large commented-out blocks in `send_from_file`: an earlier
> file-streaming variant and an in-C++ per-sample frequency-shift variant. They are
> retained as reference/history, not compiled.

### 4.8 `UHD_SAFE_MAIN` — entry point

1. **CLI parsing** via Boost `program_options` (see §6 for the full option list).
2. **Hard-coded overrides** (these currently override several CLI values):
   - `file = "/home/user/Projects/files_for_phi/tst/cw_FM.sc16"`
   - `rate = 50e6`
   - `freq = 140e6`
   - `repeat = true`
3. Create USRP, parse RX/TX channels, set clock/subdev if given.
4. Configure rate, frequency, gain (default 10 dB if `--gain` absent), optional bw/antenna.
5. Lock LOs (unless `--skip-lo`); optionally check `mimo_locked` / `ref_locked`.
6. Map `--type` → CPU format (`fc64`/`fc32`/`sc16`) and build RX + TX streamers.
7. **Spawn RX thread(s)**: one shared thread over all channels by default, or one thread
   per channel with `--multi_streamer`. The `recv_samples_args` macro packs the long
   argument list to `samplesRX`.
8. **Spawn the single TX thread** running `send_from_file`.
9. Join the TX thread, then monitor/join RX threads (optionally printing aggregate Msps
   with `--progress`) until `Ctrl+C` or the sample budget is met.

The `create_receiver` / `create_transmitter` lambdas instantiate the correctly
templated `SignalRecv<T>` / `SignalTnsmt<T>` based on `--type` and `--wirefmt`.

---

## 5. The IQ sample file format (sc16)

The transmitted files are raw **interleaved 16-bit signed integer IQ**:

```
int16 I0, int16 Q0, int16 I1, int16 Q1, ...   (little-endian, no header)
```

This matches UHD's `sc16` wire/CPU format and the `std::complex<short>` sample type.
Full-scale is ±32767. `jams.py`'s `save_to_sc16` writes exactly this layout, and
`SignalTnsmt::send_from_file` reads it back with `infile.read((char*)&jam_buff...)`.

---

## 6. Command-line options (`reactive`)

| Option | Default | Description |
|--------|---------|-------------|
| `--help` | — | Print help and exit. |
| `--args` | `""` | UHD multi-device address args. |
| `--type` | `short` | Sample type: `double`, `float`, or `short`. |
| `--nsamps` | `0` | Total samples to receive (`0` = run until Ctrl+C). |
| `--spb` | `8192` | Samples per buffer (also the FFT/WOLA block size). |
| `--rate` | `50e6` | Sample rate (Hz). **Currently overridden to `50e6` in code.** |
| `--freq` | `146e6` | RF center frequency (Hz). **Currently overridden to `140e6` in code.** |
| `--lo-offset` | `0.0` | LO offset (Hz). |
| `--gain` | `0.0` | RF chain gain (dB); defaults to 10 if unset. |
| `--ant` | — | Antenna selection. |
| `--subdev` | — | Subdevice spec. |
| `--rx_channels` | `0` | RX channel(s), e.g. `"0"`, `"0,1"`. |
| `--tx_channels` | `1` | TX channel(s). |
| `--bw` | — | Analog front-end filter bandwidth (Hz). |
| `--ref` | — | Reference source: `internal`, `external`, `mimo`. |
| `--wirefmt` | `sc16` | Wire format: `sc8`, `sc16`, or `s16`. |
| `--setup` | `1.0` | Setup/settling time (s). |
| `--progress` | — | Periodically print short-term bandwidth. |
| `--stats` | — | Print average bandwidth on exit. |
| `--sizemap` | — | Track and print packet-size breakdown. |
| `--null` | — | Run without writing to file. |
| `--continue` | — | Don't abort on a bad packet. |
| `--skip-lo` | — | Skip LO-lock checking. |
| `--int-n` | — | Use integer-N tuning. |
| `--delay` | `0.0` | Delay between repeated transmissions (s). |
| `--file` | `home/user/Projects/files_for_phi/tst/cw.sc16` | IQ file to transmit. **Currently overridden in code to `.../cw_FM.sc16`.** |
| `--multi_streamer` | — | One RX streamer/thread per channel. |
| `--repeat` | — | Repeat (forced `true` in code). |
| `--channel` | — | (Declared, unused.) |

---

## 7. `PyFiles/jams.py` — the waveform generator (RELEVANT)

Standalone Python module that creates the binary IQ files transmitted by
`reactive.cpp`, plus analysis/plotting helpers. Three classes:

### `class files(read_files_dir, write_files_dir)` — file I/O
- `read_from_sc16(filename)` — read interleaved int16 IQ, normalize to unit-scale
  `complex128`.
- `save_to_sc16(signal, filename)` — write `complex` signal as interleaved little-endian
  int16 IQ (`×32767`). **This is the format `reactive.cpp` transmits.**
- `save_to_fc32(signal, filename)` — write as interleaved float32 (`.bin`).

### `class plots(fs)` — visualization
- `show_fft`, `show_spectrogram`, `show_constellation` — matplotlib diagnostics.

### `class jam(fs, jam_duration)` — waveform synthesis
Static/utility helpers:
- `normalize_signal`, `calc_PAPR`, `adjust_signal_power`, `calc_Backoff`,
  `adjust_length`, `low_pass_filter`, `create_gaussian_noise`,
  `complex_bandpass_filter`.

Waveform generators (each optionally multiplied by band-limited Gaussian noise):
| Method | Waveform |
|--------|----------|
| `create_FM(fc, k, f_m, signal)` | FM modulation (of a sine, or a provided signal). |
| `create_chirp(f_start, f_end, chirp_duration, noise_bw)` | Linear frequency sweep. |
| `create_multi_cw(cw_list, noise_bw)` | Sum of CW tones at listed frequencies. |
| `create_cyclic(cw_list, durations_list, noise_bw)` | Time-sequenced CW tones. |
| `create_cyclic_chirp(...)` | Concatenated chirps. |
| `create_barrage(freq_ranges, noise_bw)` | Frequency-domain comb → time domain. |
| `read_WAV(filename)` | Load/resample a WAV as a modulating source. |
| `create_DTMF(tone_dur, char_list)` | DTMF dual-tone sequence. |
| `create_BPSK(bits)` / `create_QPSK(bits)` | PSK-modulated waveforms. |

**`__main__` (as configured):** `fs = 50e6`, `dur = 0.5 s`, output dir
`/home/user/Projects/files_for_phi/tst`. It builds a single CW tone at baseband 0
(`create_multi_cw([0], 6.25e3)`), an FM waveform (`create_FM(0, 6.25e3, f_m=2e3)`),
multiplies them, and saves the CW-times-FM result as **`cw_FM.sc16`** — which is exactly
the file `reactive.cpp` transmits by default. It then plots the FFT and spectrogram.

> To generate a different jamming file, edit the `__main__` block: pick a generator,
> assign it to `signal`, and call `filer.save_to_sc16(signal, "yourname")`, then point
> `reactive.cpp`'s `file` at it.

---

## 8. `PyFiles/J2S_file_creation.py` — Jam-to-Signal sweep (NOT part of current project)

Legacy/experimental script. It reads a recorded signal (`rate_15.36e6_freq_144e6.sc16`),
generates a CW jammer, and produces a sweep of files `J2S-10 … J2S10` that mix jammer +
recording at Jam-to-Signal ratios from −10 dB to +10 dB (lowering the jammer power for
negative ratios, lowering the recording power for positive ones). Useful for evaluating
detector/jammer performance vs. J2S, but **not used by the current Reactive pipeline.**
It depends on `jams.py` (`import jams`).

---

## 9. Build & run

### 9.1 Prerequisites
- UHD ≥ 4.1.0 (development headers + libs)
- Intel oneAPI IPP at `/opt/intel/oneapi/ipp/latest` (see `IPP_ROOT` in `CMakeLists.txt`)
- Boost ≥ 1.65 (`program_options`, `system`, `filesystem`, `thread`)
- Python + pybind11 + jsoncpp (declared as CMake dependencies)
- A C++20 compiler

### 9.2 Build
```bash
cd Reactive
mkdir -p build && cd build
cmake ..
make            # produces the `reactive` executable
```
`CMakeLists.txt` builds the `reactive` target from `reactive.cpp`, links `ipps` and
`ippcore`, plus UHD and Boost. (The static-linking branch still references an old
`rx_to_file` target and would need fixing if `UHD_USE_STATIC_LIBS` is turned on.)

### 9.3 Run (example)
```bash
./reactive --rate 50e6 --freq 140e6 --rx_channels 0 --tx_channels 1 \
           --gain 10 --file /path/to/jam.sc16
# Press Ctrl+C to stop.
```
Remember that `rate`, `freq`, and `file` are currently hard-coded overrides in
`UHD_SAFE_MAIN`, so CLI values for those are ignored until those lines are removed.

---

## 10. Notes, caveats & gotchas

- **Hard-coded overrides in `main`:** `file`, `rate`, and `freq` are reassigned after
  option parsing, so the corresponding CLI flags currently have no effect.
- **Global `freqs`:** shared mutable state between RX and TX. The RX↔TX handshake
  (`signal_received` + the two CVs) is what keeps access serialized; there is no
  additional lock around `freqs` itself.
- **`tx_from_file`:** defaults to `true`. Set to `false` (or wire it to a CLI flag) to
  use the in-C++ `SignalGenerator::create_cw` synthesis path instead of a file.
- **Only `Technique::CW` is implemented** for runtime synthesis; `Multi`/`Chirp` are
  stubs. The full waveform variety lives in `jams.py` (file-based path).
- **Linux-only includes:** `boost/filesystem`, `boost/process`, `boost/random` are behind
  `#ifdef __linux__`. The runtime target is Linux; the paths (`/home/user/...`, IPP at
  `/opt/intel/...`) confirm this, even though this repo copy sits on Windows.
- **`--channel` CLI option** is declared but unused.
- **Detector tuning constants** (`amp_th = -15 dB`, `min_bw = 6.25 kHz`, 3 dB peak
  window, `WOLA_ORDER = 4`) live inside `SignalRecv` and govern sensitivity/selectivity.
