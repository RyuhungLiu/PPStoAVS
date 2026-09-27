# PPStoAVS

USB PD protocol converter on **WCH CH32M030K9U7**: takes a charger that offers **PPS** and presents
**SPR AVS** to the device. No DC/DC — the charger's output is passed through a back-to-back NMOS switch.

```
PPS charger ── Type-C plug (USBPD1, Sink) ── CH32M030K9U7 ── NMOS switch ── Type-C receptacle (USBPD0, Source) ── device
```

v0.1.1 is verified on hardware. v0.2.0 adds a USB HID link on the front D+/D− for a browser-based
host tool: change the output mode and limits, and read back per-session logs.

## Pin assignment

| Pin | Port | Function | Notes |
|---|---|---|---|
| 11 | PA3 | CC4R → USBPD1 (front, Sink) | Internal Rd; also SWIO. SDI is disabled ~300 ms after boot |
| 8 / 9 | PA0 / PA1 | CC1 / CC2 → USBPD0 (back, Source) | Rp current source, 3 A (330 µA) when attached |
| 32 | PB15 | HVOD3 → NMOS gate | OUTDR=1 pulls gate low (off); OUTDR=0 high-Z, R3 pulls gate to HVCP (on) |
| 1, 2, 5, 29, 30 | HVCP, PB14, PC5, PB12, CAP2 | 2-stage charge pump | TIM2 remap 10, PWM 500 kHz; VHVCP ≈ VHV + 8.5 V |
| 21 | PA13 | ADC_IN18 | Front VBUS, 100k/10k divider (÷11) |
| 18 / 19 | PA10 / PA11 | ISP2 / ISN2 → OPA4 → ADC_IN10 | 5 mΩ low-side shunt, gain 55, bias 1.6 V |
| 12 / 13 | PB0 / PB1 | UDP / UDM | Front D+/D− → USB HID (host tool) |
| 7 | VHV | Supply | Front VBUS |

## Capability translation

Rules follow USB PD R3.2 v1.2 (§6.4.1.1, Table 3-1/3-2 notes, §4.1.3.2.4, Table 4-3):

1. **Fixed PDOs** — passed through from the charger up to the max voltage (default 20 V), each capped at the
   current limit (default 3 A). 9/12/15/20 V can be hidden from the host tool.
2. **AVS is offered only if a 15 V Fixed PDO is offered** (spec requirement). Source, in order of preference:
   - the charger's own SPR AVS APDO, passed through;
   - otherwise a PPS APDO translated to AVS: the one reaching 20 V (min ≤ 9 V) with the highest current,
     else one reaching 15 V.
3. **AVS current** — 9–15 V uses the 15 V Fixed current, 15–20 V uses the 20 V Fixed current, where each
   is `min(charger Fixed current, source current, current limit)`. The same value is written into the 15 V / 20 V Fixed PDOs.
4. **If the AVS source tops out below 20 V**, the 20 V Fixed PDO is dropped and AVS goes to 15 V only.
5. **12 V conversion (optional)** — if the charger has no 12 V Fixed PDO, offer one backed by a PPS APDO
   covering 12 V (preferred) or by the charger's AVS. Only added when a PDO slot is still free.
6. **Order** — Fixed (ascending), then AVS, then PPS (ascending max voltage); at most 7 PDOs.
   When slots run out: Fixed > AVS > PPS (higher max voltage kept) > 12 V conversion.
7. **Requests** — a Fixed request maps to the charger's Fixed PDO (or to its PPS/AVS at 12 V); an AVS
   request (100 mV steps) maps to the charger's AVS, or to a PPS request with the PPS maximum current;
   a PPS request is forwarded as-is (20 mV / 50 mA units). Voltage changes are requested in one step.
8. **Sequence** — device Request → Accept → front Request → charger PS_RDY → front VBUS within the OVP/UVP
   window for 3 consecutive samples → PS_RDY to device. PPS keep-alive every 5 s (not needed for AVS).
9. **Charger re-advertises (DPS)** — keep the device's voltage if still available and re-advertise;
   otherwise go back to 5 V and Hard Reset the device.
10. **Non-PD charger** → 5 V only.

Protection: front VBUS outside the OVP/UVP window (default ±5 %, 3 samples) or current above the OCP
threshold (default 3.5 A for 50 ms) → switch off and Hard Reset. UVP is skipped while a PPS contract is
passed through (current-limit mode).

### Output modes

| Mode | Rear Source_Capabilities |
|---|---|
| a (default) | Fixed + AVS |
| b | Fixed + all charger PPS APDOs (max voltage/current capped) |
| c | Fixed + AVS + all charger PPS APDOs |
| d | Fixed only |

The 12 V conversion can be enabled in any mode. Changing the mode or limits re-advertises to the device;
if the current contract is no longer offered, the device is Hard Reset.

Example — charger `5/9/15/20 V Fixed, PPS 5–11 V, AVS 9–20 V, PPS 4.5–21 V` in mode c:
`5/9/15/20 V Fixed, AVS 9–20 V (#6), PPS 5–11 V (#5), PPS 4.5–20 V (#7)` — 7 PDOs, so no 12 V conversion.

### Lab mode: AVS second handshake

Some chargers first advertise PPS, query the sink with `Get_Sink_Cap_Extended`, and re-advertise with SPR AVS
in place of PPS when the sink's Sink Modes has the AVS bit (bit 5). Two independent switches:

- **Front (plug)** — reply to the charger's `Get_Sink_Cap_Extended` with Sink Modes = AVS, so such chargers
  switch to AVS. Off: reply `Not_Supported`, charger keeps PPS.
- **Rear (receptacle)** — imitate that charger: offer Fixed + PPS first; after the first contract, query the device
  and re-advertise with AVS only if it declares AVS support. Applies to modes a/c; restarts when the device is unplugged.

## Host tool

Plug the front Type-C into a PC (no charger needed; the board runs from the PC's 5 V) and open
[`tools/ppstoavs.html`](tools/ppstoavs.html) in desktop Chrome or Edge (WebHID). No driver or install.

- **Status** — charger → converter → device overview (contracts, VBUS, current, switch), charger PDOs and
  the rear PDOs with the charger PDO each one comes from.
- **Settings** — mode, 12 V conversion, max voltage (15/20 V), current limit (≤ 3 A), hidden Fixed PDOs,
  OVP/UVP %, OCP mA/ms, PPS request logging. Applied immediately and saved to flash.
- **Records** — the last power sessions (up to 64 records each): charger PDOs, the PDOs offered to the device
  (logged whenever they change, attached or not), every request (device RDO → charger RDO, voltage, result),
  resets, protection trips, attach/detach. Consecutive PPS/AVS steps on the same PDO are merged into one
  record with a repeat count. Successful device PPS requests are not logged unless enabled. Export as JSON.

Device: VID `1209` / PID `0001` (pid.codes test ID), vendor usage page `0xFF00`, 64-byte reports.
Protocol is documented in [`src/host.h`](src/host.h).

Flash layout (64 KB, 128-byte pages):

| Range | Use |
|---|---|
| `0x0000–0xBEFF` | Firmware |
| `0xBF00–0xBFFF` | Settings (2 pages, alternating, CRC) |
| `0xC000–0xFFFF` | Log ring (128 pages, CRC per page) |

Flash writes stall the CPU for ~4.5 ms, so they only run when both PD ports have been idle for 300 ms.

## Risks

- **Charge pump** — TIM2 set up from datasheet DS2 §4 steps; whether CCER channels 3/4 must be enabled
  is not documented. Check HVCP ≈ VHV + 8.5 V first.
- **SDK bug** — `PWR_CTLR_ISINKEN` is defined as `0x0100` in the SDK header; the reference manual, SVD and
  the SDK's own `ch32m030_pwr.c` use bit 10. USB PD needs ISINKEN, so the code uses bit 10.
- **ADC reference** is VDD33 (3.1–3.5 V spec). The ±5 % window may trip falsely; tune `ADC_VREF_MV`.
- **No output VBUS sensing** and no active discharge (HVOD2/DSCG not connected). On detach the firmware
  keeps the switch on, returns the charger to 5 V, then switches off (≤ 650 ms), so up to ~5 V can remain
  on the receptacle instead of vSafe0V. A bleeder resistor or DSCG circuit is needed for full compliance.
- **Gate turn-off** — Vgs can briefly approach −VBUS when the gate is pulled low.
- **AVS small steps** must finish within 50 ms; this relies on the charger's PPS response time.
- **PD2.0 devices** also receive the AVS APDO in the first Source_Capabilities.
- **Rear Rp** always advertises 3 A, even with a lower current limit; OCP still applies.

## Build

The WCH SDK is not included. Recreate it from a local MounRiver Studio 2 install, then build:

```bash
bash scripts/setup_sdk.sh
"/c/MounRiver/MounRiver_Studio2/resources/app/resources/win32/others/Build_Tools/Make/bin/make.exe"
```

Output: `build/PPStoAVS.hex` / `.bin`. Prebuilt images are attached to the GitHub release.

## Flashing

The chip has no factory bootloader (no USB/UART ISP); use WCH-LinkE in single-wire mode:
SWDIO → DIO test point (PA3/SWIO), GND → GND, WCH-LinkE 5V → VBUS.

- A blank chip connects directly. Firmware disables SWIO 300 ms after boot, so to reflash first run
  **WCH-LinkUtility → Clear All Code Flash - By Power Off** (board powered from WCH-LinkE), then program.
- **Disconnect WCH-LinkE before plugging in a charger.** PA3 is also the front CC line; the probe
  corrupts BMC and the front never receives Source_Capabilities.

Diagnostic build: `make BUILD=build_diag EXTRA_DEFS=-DFE_DIAG` (front takes over PA3 at power-up, no Hard Reset).
If the front gets no Source_Capabilities within 3 s, Fixed PDOs 5.05–5.30 V carry USBPD1 state in their
current field (10 mA = 1): IF_RX_BIT count, IF_RX_BYTE count, RX_STATE bitmap, PA3 comparator toggles,
CONTROL, CONFIG >> 6.