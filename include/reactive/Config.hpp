#pragma once

// Centralized compile-time configuration for the Reactive engine.
//
// Every value here is identical to the inline literal / preprocessor macro it
// replaced; this header only gives those constants a single, named home.
// Using `constexpr` (rather than #define) keeps them typed and scoped.

namespace cfg {

// --- DSP / detection ------------------------------------------------------
inline constexpr int    kWolaOrder    = 4;        // WOLA decimation factor
inline constexpr double kAmpThreshDb  = -15.0;    // detection threshold (dB)
inline constexpr double kPeakWindowDb = 3.0;      // keep peaks within this of the max (dB)
inline constexpr double kMinBwHz      = 6.25e3;   // min spacing between detected peaks (Hz)
inline constexpr double kFullScale    = 32767.0;  // sc16 full-scale (I/Q)

// --- Waveform synthesis ---------------------------------------------------
inline constexpr int      kJamDurSec        = 10;      // jam duration per detected freq (s)
inline constexpr double   kNoiseBwHz        = 100e3;   // Gaussian-noise bandwidth (Hz)
inline constexpr double   kToneSpacingHz    = 6.25e3;  // skip CW tones closer than this (Hz)
inline constexpr int      kNoiseFilterOrder = 3;       // Butterworth LPF order
inline constexpr unsigned kNoiseSeed        = 42;      // RNG seed for reproducible noise

// --- RF operating defaults (used as the CLI default values in main) -------
inline constexpr double      kDefaultRateHz = 50e6;         // --rate
inline constexpr double      kDefaultFreqHz = 140e6;        // --freq
inline constexpr double      kDefaultGainDb = 0.0;          // --gain
inline constexpr const char* kDefaultFile   = "cw_FM.sc16"; // --file

}  // namespace cfg
