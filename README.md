# PPStoAVS

USB PD protocol converter on **WCH CH32M030K9U7**: takes a charger that offers **PPS** and presents
**SPR AVS** to the device. No DC/DC — the charger's output is passed through a back-to-back NMOS switch.

```
PPS charger ── Type-C plug (USBPD1, Sink) ── CH32M030K9U7 ── NMOS switch ── Type-C receptacle (USBPD0, Source) ── device
```

v0.1.1 is verified on hardware. v0.2.0 adds a USB HID link on the front D+/D− for a browser-based
host tool: change the output mode and limits, and read back per-session logs. v0.3.0 adds Lab mode
(AVS second handshake) and handles fast AVS/PPS requests reliably. v0.4.0 adds Lab mode AVS as PPS. v0.5.0 adds Lab modes virtual 5 A E-Marker with EPR,
EPR AVS as SPR AVS, and custom Fixed. v0.6.0 adds voltage compensation, PD info passthrough (battery and
identity) and Lab mode force PPS. v0.6.1 fixes current sensing (R compensation and OCP). v0.7.0 adds
Lab mode custom PPS, and a PDO conversion simulator and Simplified Chinese in the host tool. v0.8.0 adds an
unstable Lab UFCS front end and a redesigned host tool. v0.9.0 adds custom VID/PID (rear Source on by default, front Sink) and a 5 V/9 V start option for AVS as PPS; Lab modes AVS as PPS, EPR AVS as SPR AVS and UFCS are marked unstable. v0.10.0 adds online update: a bootloader and a Firmware tab in the host tool (first flash needs the full image). v0.10.1 builds the application with link-time optimization (36.1 KB, was 39.7 KB) and turns event recording off by default; enable it in the host tool (Settings, Records). v0.11.0 reads the e-marker of the cable on the rear port (SOP' Discover Identity, shown on the Status tab) and, when the e-marker declares 5 A, raises the default 3 A limit to 5 A (Source only, no EPR). v0.12.0 replaces the 3 A / 5 A output cable setting with a fully editable virtual e-marker (off by default) and adds an optional custom PDO list (up to 7 levels, with AVS) to the Lab force PPS mode. v0.12.1 polishes the host tool (settings that cannot work together are greyed out with the reason or blocked before Apply, number fields snap to their range on leave, scroll position kept per tab, unsaved-change warning; UFCS moved to the front group and EPR AVS to the E-Marker tab). Firmware: custom AVS only needs a 15 V Fixed (20 V AVS needs both), and custom PPS levels are narrowed to what the charger can give instead of being dropped. v0.12.3 holds off transmitting while a received frame is still waiting for its GoodCRC (suspected cause of charger Soft Resets during PPS forwarding), fixes the over-current limit at 5.35 A / 50 ms once the rear port runs at 5 A, and adds an optional "always answer cable queries" mode to the rear virtual e-marker (answers SOP' Discover Identity with no device attached and the output off).

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

### Voltage compensation

When the front contract is PPS or AVS, the voltage requested from the charger is raised to cancel the drop across the
converter and cable. A charger Fixed PDO cannot be compensated.

- **V**: adds a fixed 0–1000 mV.
- **R** (default, 15 mΩ): adds output current × 0–500 mΩ, updated every 0.5 s from the measured current.
  15 mΩ is roughly the converter's own drop, +45 mV at 3 A.

Compensation is limited to +1 V and to the source PDO's maximum voltage. It is rounded to the charger's step (PPS 20 mV,
AVS 100 mV). OVP/UVP are checked against the compensated voltage.

### PD info passthrough

On by default. The converter reads each side's PD info and answers the other side's queries from that copy
(the spec allows only 15 ms, too short to ask the other side live). Both sides must be PD 3.0.

- **Device → charger**
  - `Battery_Capabilities` and `Battery_Status`; the device's battery `Alert` is forwarded, so chargers with a screen can show
    the phone's battery.
  - `Sink_Capabilities_Extended`. Its Sink Modes describe the converter: PPS, plus AVS only with the Lab front
    second handshake. PDP is capped to the current limit.
- **Charger → device**
  - `Source_Capabilities_Extended`, `Source_Info` and `Status`.
  - PDP values are rewritten to what the rear actually offers.
- **Identity passthrough** (separate switch, off by default)
  - `Discover Identity` answers, VID/PID in the extended info, and the charger's `Manufacturer_Info` come from the
    other side.
- **Custom IDs** (exclusive with identity passthrough)
  - The rear can present a custom Source VID/PID to the device (on by default, 5A1E:30A5, an arbitrary ID).
  - The front can present a custom Sink VID/PID to the charger (off by default).
  - They are used in `Discover Identity`, the extended info and the rear `Manufacturer_Info`.
  - When neither is on and passthrough is off, the converter's own VID/PID (1209:0001) is used.

The status page shows the device's identity and battery. The identities read from both sides are also recorded.

### Lab mode: AVS second handshake

Some chargers first advertise PPS, query the sink with `Get_Sink_Cap_Extended`, and re-advertise with SPR AVS
in place of PPS when the sink's Sink Modes has the AVS bit (bit 5). Two independent switches:

- **Front (plug)** — reply to the charger's `Get_Sink_Cap_Extended` with Sink Modes = AVS, so such chargers
  switch to AVS. Off: no AVS bit (the device's Sink_Capabilities_Extended without AVS, or `Not_Supported`), charger keeps PPS.
- **Rear (receptacle)** — imitate that charger: offer Fixed + PPS first; after the first contract, query the device
  and re-advertise with AVS only if it declares AVS support. Applies to modes a/c; restarts when the device is unplugged.

### Lab mode: AVS as PPS

Offers the charger's native SPR AVS to the device as a PPS APDO up to the AVS maximum (modes b/c; a charger
PPS with the same maximum voltage takes precedence). The range starts at 9 V by default, the real AVS range; an option
starts it at 5 V as in earlier versions, in which case requests below 9 V are rejected, since AVS starts at 9 V.
Voltages are rounded to the nearest 100 mV, e.g. PPS 12.34 V → AVS 12.3 V, 12.36 V → 12.4 V. The current is the lower of
the two AVS ranges. AVS has no current limiting, so the PPS current limit is not enforced by the charger (OCP still applies).

### Lab mode: virtual 5 A E-Marker

The front answers the charger's SOP' `Discover Identity` as a passive USB-C cable rated 50 V / 5 A and EPR capable,
so chargers that check the cable can offer 5 A PDOs. The current limit can then be set up to 5 A and OCP up to 5.5 A.
When the charger's 5 V PDO has the EPR bit, the front enters EPR mode after the first contract and reads
`EPR_Source_Capabilities`, which arrives in chunks. It then uses `EPR_Request` and sends `EPR_KeepAlive` every 250 ms.
It never requests more than 20 V.
The front starts right at power-up (no debug window) so the charger's first cable query is not missed.

Chargers query the cable only after detecting Ra on the plug's VCONN pin. The hardware revision wires plug B5
(VCONN) through 1 kΩ to PA2/CC3. With the switch on, PA2 is driven low to present Ra; with it off, PA2 is high-Z.
The v0.1 PCB lacks this connection, so there only chargers that query the cable without Ra will ask.
The converter cannot check the device-side cable, which must be rated for 5 A.

### Rear cable e-marker and 5 A (v0.11.0)

Every time a device is plugged in, the rear sends Discover Identity to the cable (SOP', up to 4 tries) once the contract is set up.
It does not look at Ra first, so it also works with a single-CC solder pad that has no second CC pin (Ra on the other CC is only
shown as a hint). The host tool shows the answer on the Status tab: cable type, current (3 A / 5 A), maximum voltage, highest USB
speed, EPR bit and VID:PID. With recording on, each result (identity, no answer or NAK, with the raw VDOs) is also written to the
flash log as a "Cable e-marker" record.

- If the e-marker (Cable VDO1 bits 6:5) says 5 A, the rear limit moves from the default 3 A to 5 A and the rear Source_Capabilities are sent again.
  The charger's own current per voltage is still the ceiling, so 5 A normally appears only at 20 V (Fixed, SPR AVS 15–20 V, PPS).
  The Source never offers EPR (no Fixed above 20 V, no EPR AVS, Enter_EPR is answered with Not_Supported).
- OCP follows the new limit: at least 7/6 of it, at most 5.5 A (the current sense saturates near 5.8 A).
- Settings → Output cable: the 3 A / 5 A choice of v0.11.0 became the virtual e-marker (below). Off (default) = read the real cable.
- To get 5 A from the charger as well, use the Lab virtual E-Marker on the front (otherwise the charger offers at most 3 A).
- **Hardware limit:** an e-marker is powered from VCONN, and this MCU cannot supply it from its CC pins. A passive e-marked cable stays
  silent, so the host tool shows "no answer". The default `be_vconn_set()` (weak, in `src/be_source.c`) drives the other CC pin with 3.3 V when Ra is seen (works with the 5 A cable tested); it is called after VBUS
  comes up on every connection; a hardware revision with a VCONN switch only has to override that function and `be_vconn_available()`.

### Lab mode: EPR AVS as SPR AVS

This mode requires the E-Marker switch. With the front in EPR mode, the SPR AVS offered to the device takes its 15–20 V range
from the charger's EPR AVS, at up to 5 A as limited by PDP. EPR AVS has nothing below 15 V. Requests of exactly 9 V or 15 V go
to the charger's Fixed 9 V / 15 V, and all other 9–15 V requests are rejected. The 9–15 V current advertised is that of
the Fixed 9 V / 15 V. When this switch is on, EPR AVS takes precedence over PPS or SPR AVS as the AVS source.

### Lab mode: custom Fixed

Adds one non-standard Fixed PDO, 5.1–20.0 V in 100 mV steps and 0.5–5 A. It is made from the PPS covering that voltage,
or from the charger's SPR AVS (9 V and up) when there is no PPS. Its current is min(setting, source, current limit).
It is not offered if the charger already has that Fixed voltage, if it exceeds the max voltage, or if all 7 PDO slots
are used. It cannot be combined with 12 V conversion.

### Lab mode: custom PPS

Adds one PPS APDO with a user-set range of 3.3–21.0 V (100 mV steps) and a current of 0.5–5 A.
- **Source**
  - The charger PPS (or SPR AVS, from 9 V; requests are then rounded to 100 mV) that overlaps the range most; on equal width a PPS wins, then the higher current.
  - If nothing covers the whole range the range is narrowed to what the source can give (3.3–21 V asked, charger PPS 5–20 V: 5–20 V is offered). Not offered only when nothing overlaps (v0.12.1).
- **Limits**
  - The maximum is capped at the max voltage setting.
  - Current = min(setting, source current, current limit). The device's per-request current limit is passed to the charger.
- **Placement**
  - Offered in every mode, among the PPS PDOs by maximum voltage.
  - It takes a slot before PPS passthrough and converted Fixed PDOs, and replaces a passthrough PPS with the same maximum.
- The configuration grows to 24 bytes (host protocol v6). Earlier settings are migrated.

### Virtual e-marker (v0.12.0)

The web tool has an **E-Marker** tab for both sides. Rear = what the converter answers to the device (this section). Front = what it answers to the charger: the Lab "Virtual E-Marker" checkbox (moved to that tab) now answers with fully editable VDOs (default: passive 50 V / 5 A / EPR cable). PA2 (pin 10, plug B5/VCONN through 1 kΩ) is pulled low as Ra only when the front Cable VDO1 says 5 A; with 3 A or with the option off it floats. The front VDOs are stored in `cfg_ext_t.fcable` and use host commands `0x15` / `0x16` (20 bytes).

Settings → Output cable → "Virtual e-marker" (off by default). When on, the rear does not read the cable and answers a device's
SOP' Discover Identity (and Soft_Reset) from stored VDOs: ID Header, Cert Stat, Product, Cable VDO1 and, for an active cable (ID Header
product type 4), Cable VDO2. A card in Settings edits every field (product type, USB host/device capable, connector type, VID, XID,
PID, bcdDevice, current, maximum voltage, USB speed, plug type, termination, latency, EPR, VBUS through cable) or the raw hex of each
VDO. Defaults are a generic passive Type-C cable (VID 1209, PID 0001, 3 A, 20 V, USB 2.0, < 10 ns, no EPR); the "Generic 3 A" /
"Generic 5 A" buttons reset to them.

- The current bits (Cable VDO1 bits 6:5) decide the rear limit: 3 A (default) gives the same PDOs as the old fixed-3 A setting, 5 A the
  same as the old 5 A setting (default 3 A limit raised to 5 A, OCP at least 7/6 of it). Source still never offers EPR.
- Self Q&A: if the sink has not sent a SOP' query 300 ms after the first Source_Capabilities, the converter plays both sides once per
  attach on SOP' (port Discover Identity, cable GoodCRC, cable ACK with the virtual VDOs, port GoodCRC) so a sniffer/tester on the CC
  line can read the cable. Skipped when the sink queries by itself.
- Changing other fields is not spec-compliant. It is only for testing e-marker readers and sinks. The Status tab shows the virtual data
  as "Virtual".
- Stored in `cfg_ext_t` (52 bytes, flags + 5 cable VDOs + 7 PDOs), saved on the same Flash page as `cfg_t` (magic `PCF5`; `PCF4` pages
  load with defaults). The old `flags3` bits 1/2 (3 A / 5 A) are migrated into the virtual e-marker once. Host protocol 11: commands
  `0x13` CFGX_GET / `0x14` CFGX_SET; `0x12` restores both.

### Lab mode: force PPS

Every Fixed level that a charger PPS covers is served from that PPS instead of the charger's Fixed. The PPS with the highest current
is used first, and the Fixed current offered becomes the PPS current. AVS is translated from PPS when a PPS covers the same
range as the charger's native AVS. The front contract is then always PPS, so voltage compensation applies to every level. It is
still capped at the PPS maximum, so there is no compensation at 20 V with PPS 5–20 V. Voltages no PPS covers still use the
charger's Fixed. Works in all modes.

**Custom PDO list (v0.12.0, needs force PPS).** Off by default (the original charger PDOs pass through with the rules above). When on,
the Settings page edits a list of up to 7 PDOs: Fixed (5–20 V, 0.1 V steps, 0.5–5 A), PPS (min ≥ 3.3 V, max ≤ 21 V, 0.5–5 A) and an AVS
tick box (one slot). The 5 V Fixed is mandatory; AVS requires a 15 V Fixed in the list (AVS reaches 20 V only if a 20 V Fixed is listed too, otherwise it stops at 15 V), which the host tool checks and the
firmware re-checks. Output modes still filter the list: a keeps Fixed + AVS, b Fixed + PPS, c all, d only Fixed. Hide Fixed, 12 V
conversion and the custom Fixed / PPS options are ignored while the list is on.

- Each level is offered only if the charger can supply it. Fixed: from a charger PPS covering the voltage (highest current first), else the
  charger's own Fixed, else a native AVS from 9 V. PPS: from the charger PPS or native AVS (from 9 V) overlapping the range most; the range is narrowed to what the charger can give (v0.12.1). A Fixed voltage cannot be narrowed, so it is skipped when unreachable.
  The current is min(your value, charger source, current limit, 5 A cable rule); the maximum voltage setting still caps the list.
- AVS follows the existing rule: source = native AVS or a PPS reaching 15/20 V; 9–15 V current follows the 15 V Fixed, 15–20 V the 20 V Fixed.
- The device list is sorted Fixed (ascending), AVS, PPS. The Simulator tab shows the result (the port in `simBuild` follows `build_back_caps`).

## Host tool

Plug the front Type-C into a PC (no charger needed; the board runs from the PC's 5 V) and open
[`tools/ppstoavs.html`](tools/ppstoavs.html) in desktop Chrome or Edge (WebHID). No driver or install.

- **Status** — charger → converter → device overview (contracts, VBUS, current, switch), charger PDOs,
  the rear PDOs with the charger PDO each one comes from, and the device's identity and battery.
- **Settings** — mode, 12 V conversion, max voltage (15/20 V), current limit (≤ 3 A), hidden Fixed PDOs,
  OVP/UVP %, OCP mA/ms, PPS request logging. Applied immediately and saved to flash.
  **Export / Import settings** save the whole form (plus the charger/device PDOs and simulator rows) as a JSON
  file for bug reports; importing only fills the form, press Apply to write it to the device.
- **Firmware** — shows the host tool version, checks the latest GitHub release (version + release notes) and can
  download its application image (`dist/PPStoAVS-app.bin` at the release tag) for flashing.
- **Records** — the last power sessions (up to 64 records each): charger PDOs, the PDOs offered to the device
  (logged whenever they change, attached or not), every request (device RDO → charger RDO, voltage, result),
  resets, protection trips, attach/detach. Consecutive PPS/AVS steps on the same PDO are merged into one
  record with a repeat count. Successful device PPS requests are not logged unless enabled. Export as JSON. Recording itself is off by default (v0.10.1); turn on "Record events to flash" in Settings first.
- **Simulator** — enter charger PDOs (Fixed / PPS / SPR AVS / EPR AVS; 5 rows by default, or load the
  connected charger's) and see the PDOs the device would be offered under the current, not yet applied,
  settings. Same rules as the firmware. Works offline.

Languages: English, 繁體中文, 简体中文.

Device: VID `1209` / PID `0001` (pid.codes test ID), vendor usage page `0xFF00`, 64-byte reports.
Protocol is documented in [`src/host.h`](src/host.h).

Flash layout (64 KB, 128-byte pages):

| Range | Use |
|---|---|
| `0x0000–0x17FF` | Bootloader (6 KB) |
| `0x1800–0xBDFF` | Application |
| `0xBE00–0xBE7F` | Boot-to-bootloader flag page |
| `0xBE80–0xBEFF` | Application header (size, CRC-32, version) |
| `0xBF00–0xBFFF` | Settings (2 pages, alternating, CRC) |
| `0xC000–0xFFFF` | Log ring (128 pages, CRC per page) |

Flash writes stall the CPU for ~4.5 ms, so they only run when both PD ports have been idle for 300 ms.

### Online update (bootloader)

The bootloader runs first after every reset. It starts the application if the header and the CRC-32 are valid;
otherwise (new chip, interrupted update) it stays in bootloader mode. In bootloader mode the front D+/D− is the same
USB HID device (same VID/PID), the rear has no output and PD is off. Protocol: [`src/iap.h`](src/iap.h).

The host tool's **Firmware** tab shows the mode and flashes an application `.bin` or `.hex` (a full image works too):
*Flash* enters the bootloader, writes and verifies the file, then starts the application. The header is written last,
so an interrupted update leaves the device in the bootloader and it can be flashed again. Settings and records are kept.

First flash (or an older firmware without a bootloader): program the **full image** with WCH-LinkUtility.
Firmware v0.9.0 and older cannot be updated this way.

## UFCS front end (unstable)

Lab setting *UFCS on the front port* (off by default). At power-up the converter checks the front D+/D−; if the
charger answers the UFCS handshake (T/TAF 083-2022), the front port runs UFCS instead of PD. Its output modes are
turned into equivalent PD capabilities — Fixed 5/9/15/20 V where a mode covers them, plus one PPS per mode — and the
device's requests become UFCS Requests, so every rear mode, compensation and Lab item works unchanged.
Chargers that do not answer fall back to PD. UFCS shares D+/D− with USB HID, so the host tool cannot connect while
UFCS is active; change settings with the charger unplugged.

`make BUILD=build_ufcs EXTRA_DEFS=-DUFCS_PROBE` builds a probe image instead: it pings, reads the output modes, requests
each mode's voltage (up to 20 V) and polls Source_Information, logging raw packets and steps. The switch stays off.
Read the result in the host tool's Records tab after plugging into a PC.

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

Output: `build/PPStoAVS.hex` / `.bin` (application, linked at `0x1800`). Prebuilt images are attached to the GitHub release.

Bootloader and full image (bootloader + application + header, for the first flash):

```bash
make -f Makefile.bl
python scripts/mkfull.py        # build/PPStoAVS-full.hex / .bin
```

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