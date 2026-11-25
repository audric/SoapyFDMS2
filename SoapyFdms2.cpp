// SoapySDR module for the ELAD FDM-S2 receiver (driver name "fdms2").
//
// The FDM-S2 is a direct-sampling receiver (122.88 MHz ADC) behind a Cypress
// FX2 USB interface: vendor control requests for tuning and the front end, and
// I/Q on bulk endpoint 0x86 as interleaved little-endian 32-bit I, Q.
//
// The sample rate is not a setting: each rate is its own FPGA image, loaded
// over USB (about 6 s) and lost at power-off. The images are ELAD's and are not
// part of this repository; setup.sh fetches them into the
// firmware directory. activateStream loads the image for the chosen rate
// unless the FPGA is already running at that rate, which it finds out by
// measuring the stream.
//
// Streaming and tuning requests follow ELAD's open code (elad-local.c,
// gr-elad). The FPGA upload follows what ELAD's loader program sends:
//   0xEA in, value {0,0,1,0,3}[i], index i = 0..4, reply 0xEA   ADC set
//   0xF6 in, reply 0xF6                                         external filter
//   0xE3 in, reply 0xE3, then 1 s                               FPGA reset
//   0xE4 out, value = length, 64 bytes at a time                the image
//   0xE4 out, value 1, one byte 0xFF                            end of image
//   0xE5 in, 1 byte, non-zero = FPGA configured
// Other requests:
//   0xEB in, then 0xE8 in, 1 byte each          stop and re-initialise the sample FIFO
//   0xE9 in, value 1 / 0, reply 0xE9            start / stop the sample FIFO
//   0xF2 out, value = word low 16, index = word high 16, 2 bytes   tune
//   0xF7 in, value = filter word, index 2, reply 0xF7   preselector, attenuator
//   0xA2 in, index 0x0151, value = EEPROM address       serial 0x4000 (32 bytes),
//        clock correction 0x4024 (int32 Hz), hardware version 0x404C (2 bytes)
//   0xFF in, 2 bytes                                    USB firmware version
//
// Linux only (libusb). One stream at a time per receiver.
//
// SPDX-License-Identifier: GPL-3.0-or-later
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include <libusb.h>

#include "Fdms2Protocol.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint16_t kVid = 0x1721;
constexpr uint16_t kPid = 0x061C;
constexpr uint8_t kBulkIn = 0x86;
constexpr unsigned kUsbTimeoutMs = 2000;
constexpr int kTransfers = 16;
constexpr int kTransferBytes = 32768;        // a multiple of the 512-byte packet and of the 8-byte frame
constexpr size_t kMtu = 16384;               // frames per read
constexpr double kAttenuatorDb = 12.0;       // nominal; the EEPROM's level offset for it is 12.5 dB
constexpr const char* kFirmwareDir = "/usr/local/share/soapy-fdms2";

struct UsbContext {
    UsbContext() { if (libusb_init(&ctx) != 0) { ctx = nullptr; } }
    ~UsbContext() { if (ctx) { libusb_exit(ctx); } }
    libusb_context* ctx{nullptr};
};

int controlIn(libusb_device_handle* h, uint8_t request, uint16_t value, uint16_t index, unsigned char* data, uint16_t length)
{
    return libusb_control_transfer(h, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, request,
                                   value, index, data, length, kUsbTimeoutMs);
}

int controlOut(libusb_device_handle* h, uint8_t request, uint16_t value, uint16_t index, unsigned char* data, uint16_t length)
{
    return libusb_control_transfer(h, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT, request,
                                   value, index, data, length, kUsbTimeoutMs);
}

std::string readSerial(libusb_device_handle* h)
{
    unsigned char text[32] = {0};
    if (controlIn(h, 0xA2, 0x4000, 0x0151, text, sizeof(text)) != int(sizeof(text))) { return {}; }
    return std::string(reinterpret_cast<const char*>(text), fdms2::textLength(text, sizeof(text)));
}

// Every FDM-S2 that can be opened, with its serial. The handles stay open.
struct Found {
    libusb_device_handle* handle;
    std::string serial;
};
std::vector<Found> openAll(libusb_context* ctx)
{
    std::vector<Found> found;
    libusb_device** list = nullptr;
    const ssize_t n = libusb_get_device_list(ctx, &list);
    for (ssize_t i = 0; i < n; ++i) {
        libusb_device_descriptor desc{};
        if (libusb_get_device_descriptor(list[i], &desc) != 0) { continue; }
        if (desc.idVendor != kVid || desc.idProduct != kPid) { continue; }
        libusb_device_handle* h = nullptr;
        if (libusb_open(list[i], &h) != 0) { continue; }
        found.push_back({h, readSerial(h)});
    }
    if (list) { libusb_free_device_list(list, 1); }
    return found;
}

class SoapyFdms2 : public SoapySDR::Device
{
public:
    explicit SoapyFdms2(const SoapySDR::Kwargs& args)
    {
        if (!m_usb.ctx) { throw std::runtime_error("fdms2: libusb_init failed"); }
        const std::string wanted = args.count("serial") ? args.at("serial") : std::string();
        for (const Found& f : openAll(m_usb.ctx)) {
            if (!m_handle && (wanted.empty() || wanted == f.serial)) {
                m_handle = f.handle;
                m_serial = f.serial;
            } else {
                libusb_close(f.handle);
            }
        }
        if (!m_handle) { throw std::runtime_error("fdms2: no FDM-S2 found on USB, or no permission to open it"); }
        libusb_set_auto_detach_kernel_driver(m_handle, 1);
        const int rc = libusb_claim_interface(m_handle, 0);
        if (rc != 0) {
            libusb_close(m_handle);
            throw std::runtime_error(std::string("fdms2: the receiver is in use (") + libusb_error_name(rc) + ")");
        }
        unsigned char b[4] = {0};
        int32_t correction = 0;
        if (controlIn(m_handle, 0xA2, 0x4024, 0x0151, b, 4) == 4) { std::memcpy(&correction, b, 4); }
        // An unprogrammed EEPROM reads as all ones; a real correction is a few kHz at most.
        m_clockCorrectionHz = std::abs(correction) <= 100000 ? correction : 0;

        if (args.count("firmware_dir")) {
            m_firmwareDir = args.at("firmware_dir");
        } else if (const char* env = std::getenv("FDMS2_FIRMWARE_DIR")) {
            m_firmwareDir = env;
        }
        for (const fdms2::Rate& r : fdms2::kRates) {
            if (std::ifstream(imagePath(r)).good()) { m_rates.push_back(r.hz); }
        }
        if (m_rates.empty()) {
            SoapySDR::logf(SOAPY_SDR_WARNING, "fdms2: no FPGA image in %s; run setup.sh", m_firmwareDir.c_str());
        } else if (std::find(m_rates.begin(), m_rates.end(), m_rate) == m_rates.end()) {
            m_rate = m_rates.front();
        }
    }

    ~SoapyFdms2() override
    {
        stopStreaming();
        libusb_release_interface(m_handle, 0);
        libusb_close(m_handle);
    }

    std::string getDriverKey() const override { return "fdms2"; }
    std::string getHardwareKey() const override { return "FDM-S2"; }
    SoapySDR::Kwargs getHardwareInfo() const override
    {
        SoapySDR::Kwargs info;
        info["serial"] = m_serial;
        info["clock_correction_hz"] = std::to_string(m_clockCorrectionHz);
        info["firmware_dir"] = m_firmwareDir;
        std::lock_guard<std::mutex> lock(m_control);
        unsigned char b[2] = {0};
        if (controlIn(m_handle, 0xFF, 0, 0, b, 2) == 2) { info["usb_firmware"] = std::to_string(b[0]) + "." + std::to_string(b[1]); }
        if (controlIn(m_handle, 0xA2, 0x404C, 0x0151, b, 2) == 2) { info["hardware"] = std::to_string(b[0]) + "." + std::to_string(b[1]); }
        info["fpga"] = fpgaConfigured() ? "configured" : "not loaded";
        return info;
    }

    size_t getNumChannels(const int direction) const override { return direction == SOAPY_SDR_RX ? 1 : 0; }
    std::vector<std::string> listAntennas(const int, const size_t) const override { return {"RX"}; }
    std::string getAntenna(const int, const size_t) const override { return "RX"; }

    // One gain element, the attenuator: 0 dB (off) or -12 dB (on).
    std::vector<std::string> listGains(const int, const size_t) const override { return {"ATT"}; }
    SoapySDR::Range getGainRange(const int, const size_t, const std::string&) const override
    {
        return SoapySDR::Range(-kAttenuatorDb, 0.0, kAttenuatorDb);
    }
    SoapySDR::Range getGainRange(const int, const size_t) const override { return SoapySDR::Range(-kAttenuatorDb, 0.0, kAttenuatorDb); }
    void setGain(const int direction, const size_t channel, const double value) override { setGain(direction, channel, "ATT", value); }
    void setGain(const int, const size_t, const std::string& name, const double value) override
    {
        if (name != "ATT") { throw std::runtime_error("fdms2: unknown gain " + name); }
        std::lock_guard<std::mutex> lock(m_control);
        m_attenuator = value <= -kAttenuatorDb / 2.0;
        if (fpgaConfigured() && !applyFrontEnd()) { throw std::runtime_error("fdms2: setting the attenuator failed"); }
    }
    double getGain(const int, const size_t) const override { return m_attenuator ? -kAttenuatorDb : 0.0; }
    double getGain(const int, const size_t, const std::string&) const override { return m_attenuator ? -kAttenuatorDb : 0.0; }

    void setFrequency(const int, const size_t, const std::string& name, const double frequency, const SoapySDR::Kwargs&) override
    {
        if (name != "RF") { return; }
        std::lock_guard<std::mutex> lock(m_control);
        m_frequency = std::clamp(frequency, fdms2::kMinHz, fdms2::kMaxHz);
        // Before the FPGA is loaded there is nothing to tune: activateStream applies it.
        if (fpgaConfigured() && !applyFrontEnd()) { throw std::runtime_error("fdms2: setting the frequency failed"); }
    }
    double getFrequency(const int, const size_t, const std::string& name) const override { return name == "RF" ? m_frequency.load() : 0.0; }
    std::vector<std::string> listFrequencies(const int, const size_t) const override { return {"RF"}; }
    SoapySDR::RangeList getFrequencyRange(const int, const size_t, const std::string& name) const override
    {
        return name == "RF" ? SoapySDR::RangeList{SoapySDR::Range(fdms2::kMinHz, fdms2::kMaxHz)} : SoapySDR::RangeList{};
    }

    std::vector<double> listSampleRates(const int, const size_t) const override { return {m_rates.begin(), m_rates.end()}; }
    SoapySDR::RangeList getSampleRateRange(const int, const size_t) const override
    {
        SoapySDR::RangeList out;
        for (const unsigned r : m_rates) { out.emplace_back(r, r); }
        return out;
    }
    void setSampleRate(const int, const size_t, const double rate) override
    {
        if (m_rates.empty()) { throw std::runtime_error("fdms2: no FPGA image installed in " + m_firmwareDir); }
        unsigned best = m_rates.front();
        for (const unsigned r : m_rates) {
            if (std::abs(double(r) - rate) < std::abs(double(best) - rate)) { best = r; }
        }
        std::lock_guard<std::mutex> lock(m_control);
        if (m_streaming && best != m_rate) { throw std::runtime_error("fdms2: the sample rate cannot change while streaming"); }
        m_rate = best;
    }
    double getSampleRate(const int, const size_t) const override { return m_rate; }

    std::vector<std::string> getStreamFormats(const int, const size_t) const override { return {SOAPY_SDR_CF32, SOAPY_SDR_CS16}; }
    std::string getNativeStreamFormat(const int, const size_t, double& fullScale) const override
    {
        fullScale = 1.0;
        return SOAPY_SDR_CF32;
    }

    SoapySDR::Stream* setupStream(const int direction, const std::string& format, const std::vector<size_t>& channels,
                                  const SoapySDR::Kwargs&) override
    {
        if (direction != SOAPY_SDR_RX) { throw std::runtime_error("fdms2: receive only"); }
        if (channels.size() > 1 || (channels.size() == 1 && channels[0] != 0)) { throw std::runtime_error("fdms2: only channel 0 exists"); }
        if (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32) { throw std::runtime_error("fdms2: stream format must be CF32 or CS16"); }
        std::lock_guard<std::mutex> lock(m_control);
        if (m_open) { throw std::runtime_error("fdms2: a stream is already open"); }
        m_open = true;
        m_float = format == SOAPY_SDR_CF32;
        return reinterpret_cast<SoapySDR::Stream*>(this);
    }

    void closeStream(SoapySDR::Stream*) override
    {
        stopStreaming();
        std::lock_guard<std::mutex> lock(m_control);
        m_open = false;
    }

    size_t getStreamMTU(SoapySDR::Stream*) const override { return kMtu; }

    int activateStream(SoapySDR::Stream*, const int, const long long, const size_t) override
    {
        std::lock_guard<std::mutex> lock(m_control);
        if (!m_open) { return SOAPY_SDR_STREAM_ERROR; }
        if (m_streaming) { return 0; }
        const fdms2::Rate* rate = fdms2::rateFor(m_rate);
        if (!rate) { return SOAPY_SDR_STREAM_ERROR; }
        m_scale = fdms2::sampleScale(rate->rescaleDb);
        try {
            if (!runningAt(*rate)) {
                loadFpga(*rate);
                if (!startFifo()) { throw std::runtime_error("the receiver did not start streaming"); }
                startReader(*rate);
            }
            if (!applyFrontEnd()) { throw std::runtime_error("tuning failed"); }
        } catch (const std::exception& e) {
            stopReader();
            SoapySDR::logf(SOAPY_SDR_ERROR, "fdms2: %s", e.what());
            return SOAPY_SDR_STREAM_ERROR;
        }
        clearQueue();
        m_streaming = true;
        return 0;
    }

    int deactivateStream(SoapySDR::Stream*, const int, const long long) override
    {
        stopStreaming();
        return 0;
    }

    int readStream(SoapySDR::Stream*, void* const* buffs, const size_t numElems, int& flags, long long&, const long timeoutUs) override
    {
        flags = 0;
        std::unique_lock<std::mutex> lock(m_queueMutex);
        const auto ready = [this] { return m_queued >= 2 || m_dead.load() || !m_reading.load(); };
        if (!m_queueReady.wait_for(lock, std::chrono::microseconds(std::max(1000L, timeoutUs)), ready)) { return SOAPY_SDR_TIMEOUT; }
        if (m_dead) { return SOAPY_SDR_STREAM_ERROR; }
        if (m_queued < 2) {
            // not streaming: wait out the caller's timeout so a tight read loop does not spin
            m_queueReady.wait_for(lock, std::chrono::microseconds(std::max(1000L, timeoutUs)), [this] { return m_reading.load(); });
            return SOAPY_SDR_TIMEOUT;
        }
        if (m_overflow) {
            m_overflow = false;
            return SOAPY_SDR_OVERFLOW;
        }
        const size_t frames = std::min({numElems, kMtu, m_queued / 2});
        const float scale = m_scale;
        if (m_float) {
            auto* out = static_cast<float*>(buffs[0]);
            for (size_t k = 0; k < frames * 2; ++k) { out[k] = float(pop()) * scale; }
        } else {
            auto* out = static_cast<int16_t*>(buffs[0]);
            for (size_t k = 0; k < frames * 2; ++k) { out[k] = fdms2::toS16(pop(), scale); }
        }
        return static_cast<int>(frames);
    }

private:
    std::string imagePath(const fdms2::Rate& rate) const { return m_firmwareDir + "/" + rate.image; }

    // m_control held by the caller for everything from here to the queue.
    bool fpgaConfigured() const
    {
        unsigned char b = 0;
        return controlIn(m_handle, 0xE5, 0, 0, &b, 1) == 1 && b != 0;
    }

    bool applyFrontEnd()
    {
        const double hz = m_frequency;
        const uint32_t word = fdms2::tuningWord(hz, m_clockCorrectionHz);
        unsigned char b[2] = {static_cast<unsigned char>(word >> 16), 0};
        if (controlOut(m_handle, 0xF2, word & 0xFFFF, word >> 16, b, 2) != 2) { return false; }
        unsigned char reply = 0;
        return controlIn(m_handle, 0xF7, fdms2::filterWord(hz, m_attenuator), 0x0002, &reply, 1) == 1 && reply == 0xF7;
    }

    // Stops and re-initialises the FX2's sample FIFO, then starts it. Without
    // the re-initialisation a FIFO that was stopped mid-stream, or started on
    // an empty FPGA, stays silent even after a new image is loaded (seen on
    // hardware); ELAD's loader only does it when status bit 0 is set.
    bool startFifo()
    {
        unsigned char reply = 0;
        if (controlIn(m_handle, 0xEB, 0, 0, &reply, 1) != 1) { return false; }
        if (controlIn(m_handle, 0xE8, 0, 0, &reply, 1) != 1) { return false; }
        reply = 0;
        return controlIn(m_handle, 0xE9, 1, 0, &reply, 1) == 1 && reply == 0xE9;
    }

    // True when the FPGA is configured and delivers samples at this rate; the
    // reader is then left running. Otherwise everything is stopped again.
    bool runningAt(const fdms2::Rate& rate)
    {
        if (!fpgaConfigured() || !startFifo()) { return false; }
        startReader(rate);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));   // let the first transfers land
        const uint64_t before = m_bytesIn;
        const auto t0 = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const fdms2::Rate* measured = fdms2::rateFromMeasured(double(m_bytesIn - before) / 8.0 / seconds);
        if (measured && measured->hz == rate.hz && !m_dead) { return true; }
        stopReader();
        return false;
    }

    void loadFpga(const fdms2::Rate& rate)
    {
        std::ifstream in(imagePath(rate), std::ios::binary);
        std::vector<unsigned char> image((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        if (image.size() < 100000) { throw std::runtime_error("FPGA image " + imagePath(rate) + " is missing; run setup.sh"); }
        SoapySDR::logf(SOAPY_SDR_INFO, "fdms2: loading the %u S/s FPGA image (about 6 s)", rate.hz);
        unsigned char reply = 0;
        controlIn(m_handle, 0xE9, 0, 0, &reply, 1);   // stop the FIFO; fails harmlessly on an empty FPGA
        const unsigned char adc[5] = {0, 0, 1, 0, 3};
        for (uint16_t i = 0; i < 5; ++i) {
            reply = 0;
            if (controlIn(m_handle, 0xEA, adc[i], i, &reply, 1) != 1 || reply != 0xEA) { throw std::runtime_error("FPGA load: ADC setup refused"); }
        }
        reply = 0;
        if (controlIn(m_handle, 0xF6, 0, 0, &reply, 1) != 1 || reply != 0xF6) { throw std::runtime_error("FPGA load: filter setup refused"); }
        reply = 0;
        if (controlIn(m_handle, 0xE3, 0, 0, &reply, 1) != 1 || reply != 0xE3) { throw std::runtime_error("FPGA load: reset refused"); }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        for (size_t sent = 0; sent < image.size();) {
            const int n = static_cast<int>(std::min<size_t>(64, image.size() - sent));
            if (controlOut(m_handle, 0xE4, static_cast<uint16_t>(n), 0, image.data() + sent, static_cast<uint16_t>(n)) != n) {
                throw std::runtime_error("FPGA load: upload failed at byte " + std::to_string(sent));
            }
            sent += static_cast<size_t>(n);
        }
        unsigned char last = 0xFF;
        if (controlOut(m_handle, 0xE4, 1, 0, &last, 1) != 1) { throw std::runtime_error("FPGA load: upload did not finish"); }
        if (!fpgaConfigured()) { throw std::runtime_error("FPGA load: the FPGA did not accept the image"); }
    }

    static void LIBUSB_CALL onTransfer(libusb_transfer* t) { static_cast<SoapyFdms2*>(t->user_data)->transferDone(t); }

    // On the reader thread.
    void transferDone(libusb_transfer* t)
    {
        if (t->status == LIBUSB_TRANSFER_COMPLETED) {
            m_bytesIn += static_cast<uint64_t>(t->actual_length);
            push(t->buffer, static_cast<size_t>(t->actual_length) / 8 * 2);
        } else if (t->status == LIBUSB_TRANSFER_NO_DEVICE || t->status == LIBUSB_TRANSFER_ERROR || t->status == LIBUSB_TRANSFER_STALL) {
            markDead();
        }
        // A time-out only means the FIFO is idle: keep the transfer going.
        if (m_reading && !m_dead) {
            if (libusb_submit_transfer(t) == 0) { return; }
            markDead();
        }
        --m_pending;
    }

    void startReader(const fdms2::Rate& rate)
    {
        {
            std::lock_guard<std::mutex> lock(m_queueMutex);
            m_ring.assign(size_t(rate.hz) * 2, 0);   // one second of samples
            m_head = m_queued = 0;
            m_overflow = false;
            m_dead = false;
            m_reading = true;
        }
        m_bytesIn = 0;
        m_buffers.assign(kTransfers, std::vector<unsigned char>(kTransferBytes));
        for (int i = 0; i < kTransfers; ++i) {
            libusb_transfer* t = libusb_alloc_transfer(0);
            libusb_fill_bulk_transfer(t, m_handle, kBulkIn, m_buffers[size_t(i)].data(), kTransferBytes, &SoapyFdms2::onTransfer, this, 1000);
            m_transfers.push_back(t);
            if (libusb_submit_transfer(t) == 0) { ++m_pending; }
        }
        m_reader = std::thread([this] {
            while (m_reading || m_pending > 0) {
                timeval tv{0, 100000};
                libusb_handle_events_timeout(m_usb.ctx, &tv);
            }
        });
    }

    void stopReader()
    {
        if (!m_reader.joinable()) { return; }
        {
            std::lock_guard<std::mutex> lock(m_queueMutex);
            m_reading = false;
        }
        m_queueReady.notify_all();
        for (libusb_transfer* t : m_transfers) { libusb_cancel_transfer(t); }
        m_reader.join();
        for (libusb_transfer* t : m_transfers) { libusb_free_transfer(t); }
        m_transfers.clear();
        unsigned char reply = 0;
        controlIn(m_handle, 0xE9, 0, 0, &reply, 1);
    }

    void stopStreaming()
    {
        std::lock_guard<std::mutex> lock(m_control);
        stopReader();
        m_streaming = false;
    }

    void markDead()
    {
        {
            std::lock_guard<std::mutex> lock(m_queueMutex);
            m_dead = true;
        }
        m_queueReady.notify_all();
    }

    void clearQueue()
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_head = m_queued = 0;
        m_overflow = false;
    }

    // values 32-bit samples (I, Q, I, Q...) from a USB buffer. A full ring drops the new samples.
    void push(const unsigned char* data, size_t values)
    {
        {
            std::lock_guard<std::mutex> lock(m_queueMutex);
            const size_t room = m_ring.size() - m_queued;
            if (values > room) {
                m_overflow = true;
                values = room / 2 * 2;
            }
            size_t at = (m_head + m_queued) % m_ring.size();
            for (size_t k = 0; k < values; ++k) {
                std::memcpy(&m_ring[at], data + 4 * k, 4);   // little-endian host
                at = at + 1 == m_ring.size() ? 0 : at + 1;
            }
            m_queued += values;
        }
        m_queueReady.notify_one();
    }

    // m_queueMutex held, m_queued > 0.
    int32_t pop()
    {
        const int32_t v = m_ring[m_head];
        m_head = m_head + 1 == m_ring.size() ? 0 : m_head + 1;
        --m_queued;
        return v;
    }

    UsbContext m_usb;
    libusb_device_handle* m_handle{nullptr};
    std::string m_serial;
    std::string m_firmwareDir{kFirmwareDir};
    int m_clockCorrectionHz{0};
    std::vector<unsigned> m_rates;   // rates with an installed image

    // Serialises control requests and the stream's life cycle. Never taken on the reader thread.
    mutable std::mutex m_control;
    std::atomic<double> m_frequency{7.1e6};
    std::atomic<bool> m_attenuator{false};
    std::atomic<unsigned> m_rate{768000};
    bool m_open{false};
    bool m_streaming{false};
    std::atomic<bool> m_float{true};
    std::atomic<float> m_scale{1.0f};

    std::thread m_reader;
    std::vector<libusb_transfer*> m_transfers;
    std::vector<std::vector<unsigned char>> m_buffers;
    std::atomic<int> m_pending{0};
    std::atomic<uint64_t> m_bytesIn{0};

    std::mutex m_queueMutex;
    std::condition_variable m_queueReady;
    std::vector<int32_t> m_ring;
    size_t m_head{0};
    size_t m_queued{0};
    bool m_overflow{false};
    std::atomic<bool> m_dead{false};      // written under m_queueMutex
    std::atomic<bool> m_reading{false};   // written under m_queueMutex
};

std::vector<SoapySDR::Kwargs> findFdms2(const SoapySDR::Kwargs& args)
{
    std::vector<SoapySDR::Kwargs> results;
    UsbContext usb;
    if (!usb.ctx) { return results; }
    for (const Found& f : openAll(usb.ctx)) {
        libusb_close(f.handle);
        if (args.count("serial") && args.at("serial") != f.serial) { continue; }
        SoapySDR::Kwargs dev;
        dev["driver"] = "fdms2";
        dev["label"] = f.serial.empty() ? std::string("ELAD FDM-S2") : "ELAD FDM-S2 :: " + f.serial;
        dev["serial"] = f.serial;
        results.push_back(dev);
    }
    return results;
}

SoapySDR::Device* makeFdms2(const SoapySDR::Kwargs& args) { return new SoapyFdms2(args); }

SoapySDR::Registry registerFdms2("fdms2", &findFdms2, &makeFdms2, SOAPY_SDR_ABI_VERSION);

} // namespace
