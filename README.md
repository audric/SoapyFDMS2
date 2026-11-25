# SoapySDR driver for the ELAD FDM-S2

A SoapySDR module (`fdms2Support`, driver `fdms2`) for the ELAD FDM-S2, a
direct-sampling USB receiver. Tuning, the attenuator and the I/Q stream go
through libusb; no ELAD library or program is needed at run time. Linux only.

## FPGA images

The receiver's FPGA is empty at power-on, and each sample rate is its own FPGA
image. The images are ELAD's and are not in this repository.
`setup.sh` downloads ELAD's loader program from eladit.com,
cuts the images out of it, checks their hashes and installs them in
`/usr/local/share/soapy-fdms2` (`fdms2-192k.bit` ... `fdms2-3072k.bit`). The
driver offers the rates whose image is installed.

Starting a stream loads the image for the chosen rate, which takes about 6 s,
unless the FPGA is already running at that rate (the driver measures the
stream for 0.4 s to find out). The image stays loaded until the receiver loses
power.

## Build and install

    ./setup.sh --install            # add --server to share it with SoapyRemote

Run `./setup.sh` with no options to see the commands it would run. It installs
the build packages (Debian and Ubuntu), builds and installs the module, fetches
the FPGA images, writes a udev rule and ends by listing the receiver.

The module alone, by hand (the FPGA images are then still missing):

    sudo apt install libsoapysdr-dev libusb-1.0-0-dev cmake g++
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build
    sudo cmake --install build

Run `SoapySDRUtil --find="driver=fdms2"` to check. Pass
`-DFDMS2_BUILD_TESTS=OFF` to skip the protocol test (`build/fdms2_protocol_test`).

## Use

- Device arguments: `driver=fdms2`, `serial=<serial>` (the serial in the
  receiver's EEPROM), `firmware_dir=<directory>` (where the FPGA images are;
  the environment variable `FDMS2_FIRMWARE_DIR` does the same).
- Sample rates: 192000, 384000, 768000, 1536000 and 3072000 S/s. Set the rate
  before activating the stream: it is fixed while streaming.
- Stream formats: CF32 (native, full scale 1.0) and CS16 (the receiver's 32
  bits rounded to 16).
- Tuning 9 kHz to 160 MHz. The preselector follows the frequency (below
  61.44 MHz, 61.44 to 122.88 MHz, above). The EEPROM's clock correction is
  applied to the tuning.
- Gain element `ATT`: 0 dB, or -12 dB with the attenuator in.
- The process needs write access to the receiver's USB node: the setup script
  installs a udev rule for the `plugdev` group (1721:061c).

## Status

Written and tried on one receiver (hardware 1.1, USB firmware 3.0) on a
Raspberry Pi 5 (Debian 13, arm64), locally and through SoapyRemote, on HF.
Above 61.44 MHz the receiver undersamples; that range is untried. The 6144 kS/s
rate, the FDM-S1 and the FDM-DUO are not supported.

Streaming and tuning follow ELAD's open code (elad-local.c, gr-elad). The FPGA
upload is not a published interface: the sequence is the one ELAD's loader
program sends. ELAD does not support this driver.

License: GPL-3.0-or-later (this driver; not the FPGA images).
