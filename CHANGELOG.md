# Changelog

Newest first. Downloads and SHA-256 sums are on the [Releases](https://github.com/RyuhungLiu/PPStoAVS/releases) page.

## v0.16.3
- MPL-2.0 license for firmware and host tool (README License and References sections, SPDX headers).
- `src/pd_defs.h` message and PDO definitions rewritten from the USB PD R3.2 specification; binary unchanged.
- Host tool, Simplified Chinese: event names in the log and exported JSON were still Traditional; the character table
  now covers about 2,400 common characters plus mainland terms.

## v0.16.2
- Host tool: request tags run 1–255 and never use 0, which is left to the polling of the Android notification service.

## v0.16.1
- New board: UFCS runs on PB5/PB4 (TIM1 + DMA transmit), so the host tool works on the rear port while the front runs
  UFCS. QC and AFC still need the PB0/PB1 DAC.
- Rear USB is restarted when the rear device is unplugged, so a phone or Type-C computer is asked again to become USB host.

## v0.16.0
- Front virtual 5 A e-marker works with chargers that query the cable right after VBUS turns on (verified: the charger
  offers 5 A). Bootloader 2.2 starts the application after checking only the header (the full CRC took about 85 ms at
  the 8 MHz reset clock); the application checks its CRC in the background.
- With the front e-marker on, no log pages are erased at boot.
- The e-marker needs a fixed 1 kΩ Ra from the plug's B5 (VCONN) to GND.
- Host tool: the current limit follows the front e-marker (5 A ↔ 3 A).

## v0.15.0
- New board with a USB switch (TPCH442EG, SEL on PA12): if no computer uses the front port within 3 s, USB moves to the
  rear port. Old boards are detected and stay on the front.
- Rear DR_Swap: a phone or Type-C computer on the rear is asked to become USB host; a device's DR_Swap is accepted.
- Bootloader 2.1 follows the application to the rear port.
- Host tool header shows the rear output (`PPS:33W` style, plus measured power).

## v0.14.6
- PPS pass-through never asks the charger for more current than its current PPS offers.
- Firmware version encoded as major<<12 | minor<<6 | patch (the host tool reads both formats).
- QC probe image removed.

## v0.14.5
- Lab Samsung AFC front end: the charger's voltage/current table becomes Fixed levels; switching uses the D− single-wire
  bus (`src/hisi_phy.c`).

## v0.14.0
- Flash layout 2: application area 41.5 KB → 54.5 KB, event log 126 → 32 pages, bootloader 2.0 (boards on v0.13.x or
  older need one WCH-LinkE flash of the full image).
- Lab QC 2.0/3.0 front end (after UFCS; UFCS wins when both are on): Fixed 5/9/12 V plus QC3 as PPS, optionally one
  3.6–12 V PPS whose current follows the voltage band.
- PPS contracts are kept when a capability's current drops; capabilities are re-sent instead of a Hard Reset.

## v0.13.0
- Second custom PDO list for Lab force PPS, offered once the device declares AVS in Sink_Capabilities_Extended.
- A port with only an Rd pull-down and no PD reply gets 16 Source_Capabilities before it is treated as non-PD.

## v0.12.3
- Transmission waits while a received frame still waits for its GoodCRC (suspected cause of Soft Resets during PPS).
- OCP fixed at 5.35 A / 50 ms once the rear runs at 5 A.
- Rear virtual e-marker can always answer cable queries, even with no device attached.

## v0.12.1
- Host tool polish: conflicting settings greyed out with the reason, number fields snap to range, per-tab scroll,
  unsaved-change warning.
- Custom AVS only needs a 15 V Fixed; custom PPS ranges are narrowed instead of dropped.

## v0.12.0
- Fully editable virtual e-marker (replaces the 3 A / 5 A output cable setting).
- Optional custom PDO list (up to 7 levels, with AVS) in Lab force PPS.

## v0.11.0
- Reads the rear cable's e-marker; a 5 A cable raises the default 3 A limit to 5 A (Source only, no EPR).

## v0.10.1
- Link-time optimization (36.1 KB, was 39.7 KB); event recording off by default.

## v0.10.0
- Online update: bootloader and a Firmware tab in the host tool (first flash needs the full image).

## v0.9.0
- Custom VID/PID (rear Source on by default, front Sink); 5 V / 9 V start option for AVS as PPS.

## v0.8.0
- Lab UFCS front end (unstable) and a redesigned host tool.

## v0.7.0
- Lab custom PPS, PDO conversion simulator, Simplified Chinese in the host tool.

## v0.6.1
- Current sensing fix (R compensation and OCP).

## v0.6.0
- Voltage compensation, PD info passthrough (battery and identity), Lab force PPS.

## v0.5.0
- Lab virtual 5 A e-marker with EPR, EPR AVS as SPR AVS, custom Fixed.

## v0.4.0
- Lab AVS as PPS.

## v0.3.0
- Lab AVS second handshake; fast AVS/PPS requests handled reliably.

## v0.2.0
- USB HID link on the front D+/D− and the browser host tool: output mode, limits, per-session logs.

## v0.1.1
- First version verified on hardware.
