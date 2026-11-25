// Pure rules of the ELAD FDM-S2 USB protocol: no USB, no SoapySDR, so they can
// be tested on their own. Requests and constants follow ELAD's open streaming
// code (elad-local.c, gr-elad's fdm_source_c_impl.cc).
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fdms2 {

inline constexpr double kAdcClockHz = 122880000.0;
inline constexpr double kMinHz = 9e3;
inline constexpr double kMaxHz = 160e6;

// One FPGA image per sample rate. rescaleDb is ELAD's level trim for the image.
struct Rate {
    unsigned hz;
    const char* image;   // file name in the firmware directory
    double rescaleDb;
};
// 6144000 S/s exists too (16-bit samples, the limit of USB 2.0); not offered.
inline constexpr Rate kRates[] = {
    {192000, "fdms2-192k.bit", 0.0},   {384000, "fdms2-384k.bit", 6.0},   {768000, "fdms2-768k.bit", 6.0},
    {1536000, "fdms2-1536k.bit", 5.4}, {3072000, "fdms2-3072k.bit", 5.4},
};

inline const Rate* rateFor(unsigned hz)
{
    for (const Rate& r : kRates) {
        if (r.hz == hz) { return &r; }
    }
    return nullptr;
}

// The listed rate nearest to hz.
inline const Rate& nearestRate(double hz)
{
    const Rate* best = &kRates[0];
    for (const Rate& r : kRates) {
        if (std::fabs(r.hz - hz) < std::fabs(best->hz - hz)) { best = &r; }
    }
    return *best;
}

// The listed rate a measured stream is running at: within 20 %, else nullptr.
inline const Rate* rateFromMeasured(double samplesPerSecond)
{
    for (const Rate& r : kRates) {
        if (std::fabs(samplesPerSecond - r.hz) <= 0.2 * r.hz) { return &r; }
    }
    return nullptr;
}

// The 32-bit NCO word for hz. clockCorrectionHz is the EEPROM's correction to
// the 122.88 MHz ADC clock. Frequencies above the clock alias down (hz mod clock).
inline uint32_t tuningWord(double hz, int clockCorrectionHz)
{
    const double clock = kAdcClockHz + clockCorrectionHz;
    const double f = hz - std::floor(hz / clock) * clock;
    const double word = std::floor(4294967296.0 * f / clock + 0.5);
    return word >= 4294967296.0 ? 0u : static_cast<uint32_t>(word);
}

// wValue of the filter request: the preselector for hz, plus the attenuator bit.
inline uint16_t filterWord(double hz, bool attenuator)
{
    uint16_t fl = hz < kAdcClockHz / 2.0 ? 0x03 : hz < kAdcClockHz ? 0x02 : 0x04;
    if (attenuator) { fl |= 0x08; }
    return fl;
}

// Multiplies a raw 32-bit sample to float full scale 1.0, with the image's trim.
inline float sampleScale(double rescaleDb)
{
    return static_cast<float>(std::pow(10.0, rescaleDb / 20.0) / 2147483648.0);
}

// A 32-bit sample as 16 bit (the top 16 bits, rounded, saturated) after the same trim.
inline int16_t toS16(int32_t raw, float scale)
{
    const float v = std::nearbyint(float(raw) * scale * 32768.0f);
    return static_cast<int16_t>(v > 32767.0f ? 32767.0f : v < -32768.0f ? -32768.0f : v);
}

// An EEPROM text field: up to n bytes, cut at the first NUL, trailing blanks dropped.
inline size_t textLength(const unsigned char* text, size_t n)
{
    size_t len = 0;
    while (len < n && text[len] != 0) { ++len; }
    while (len > 0 && (text[len - 1] == ' ' || text[len - 1] == 0xFF)) { --len; }
    return len;
}

} // namespace fdms2
