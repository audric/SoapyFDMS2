// SPDX-License-Identifier: GPL-3.0-or-later
#include "Fdms2Protocol.h"

#include <cstdio>
#include <cstring>

static int failures = 0;
static void check(bool ok, const char* label)
{
    if (!ok) { std::printf("FAIL: %s\n", label); ++failures; }
}

int main()
{
    using namespace fdms2;
    // The word ELAD's own code computes for 7.1 MHz with a -956 Hz clock correction (seen on hardware).
    check(tuningWord(7100000.0, -956) == 0x0ECAB235u || tuningWord(7100000.0, -956) == 0x0ECAB236u, "7.1 MHz word");
    check(tuningWord(0.0, 0) == 0u, "0 Hz word");
    check(tuningWord(kAdcClockHz / 2.0, 0) == 0x80000000u, "half the clock");
    check(tuningWord(kAdcClockHz + 1e6, 0) == tuningWord(1e6, 0), "above the clock aliases down");
    check(tuningWord(kAdcClockHz - 0.001, 0) == 0u, "a word that rounds to 2^32 wraps to 0");

    check(filterWord(7.1e6, false) == 0x03, "HF filter");
    check(filterWord(7.1e6, true) == 0x0B, "HF filter with attenuator");
    check(filterWord(100e6, false) == 0x02, "second zone filter");
    check(filterWord(145e6, false) == 0x04, "third zone filter");
    check(filterWord(61.44e6, false) == 0x02, "zone boundary belongs to the upper zone");

    check(rateFor(768000) && std::strcmp(rateFor(768000)->image, "fdms2-768k.bit") == 0, "768k image");
    check(rateFor(6144000) == nullptr && rateFor(48000) == nullptr, "unlisted rates");
    check(nearestRate(800000.0).hz == 768000, "nearest rate");
    check(nearestRate(1.0).hz == 192000 && nearestRate(1e9).hz == 3072000, "nearest rate at the ends");
    check(rateFromMeasured(770048.0) && rateFromMeasured(770048.0)->hz == 768000, "measured 768k");
    check(rateFromMeasured(385024.0) && rateFromMeasured(385024.0)->hz == 384000, "measured 384k");
    check(rateFromMeasured(0.0) == nullptr && rateFromMeasured(550000.0) == nullptr, "no rate between the steps");

    const float unity = sampleScale(0.0);
    check(std::fabs(float(INT32_MIN) * unity + 1.0f) < 1e-6f, "full scale is 1.0");
    check(std::fabs(sampleScale(6.0) / unity - 1.9953f) < 1e-3f, "6 dB trim");
    check(toS16(0x7FFFFFFF, unity) == 32767 && toS16(INT32_MIN, unity) == -32768, "16 bit ends");
    check(toS16(0x00018000, unity) == 2, "16 bit rounds");
    check(toS16(0x7FFFFFFF, sampleScale(6.0)) == 32767, "16 bit saturates with the trim");

    const unsigned char padded[8] = {'S', 'E', '0', '1', ' ', ' ', 0, 'x'};
    check(textLength(padded, 8) == 4, "serial is cut at NUL and blanks");
    const unsigned char full[4] = {'A', 'B', 'C', 'D'};
    check(textLength(full, 4) == 4, "serial without a terminator");

    if (failures == 0) { std::printf("PASS\n"); }
    return failures == 0 ? 0 : 1;
}
