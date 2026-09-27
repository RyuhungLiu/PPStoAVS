# PPStoAVS

USB PD protocol converter on **WCH CH32M030K9U7**: takes a charger that offers **PPS** and presents
**SPR AVS** to the device. No DC/DC — the charger's output is passed through a back-to-back NMOS switch.

```
PPS charger ── Type-C plug (USBPD1, Sink) ── CH32M030K9U7 ── NMOS switch ── Type-C receptacle (USBPD0, Source) ── device
```

First version: minimal, offline (no logging), **not yet verified on hardware**.

## Pin assignment

| Pin | Port | Function | Notes |
|---|---|---|---|
| 11 | PA3 | CC4R → USBPD1 (front, Sink) | Internal Rd; also SWIO. SDI is disabled ~300 ms after boot |
| 8 / 9 | PA0 / PA1 | CC1 / CC2 → USBPD0 (back, Source) | Rp current source, 3 A (330 µA) when attached |
| 32 | PB15 | HVOD3 → NMOS gate | OUTDR=1 pulls gate low (off); OUTDR=0 high-Z, R3 pulls gate to HVCP (on) |
| 1, 2, 5, 29, 30 | HVCP, PB14, PC5, PB12, CAP2 | 2-stage charge pump | TIM2 remap 10, PWM 500 kHz; VHVCP ≈ VHV + 8.5 V |
| 21 | PA13 | ADC_IN18 | Front VBUS, 100k/10k divider (÷11) |
| 18 / 19 | PA10 / PA11 | ISP2 / ISN2 → OPA4 → ADC_IN10 | 5 mΩ low-side shunt, gain 55, bias 1.6 V |
| 12 / 13 | PB0 / PB1 | UDP / UDM | Front D+/D−, reserved |
| 7 | VHV | Supply | Front VBUS |

## PPS → AVS translation

Rules follow USB PD R3.2 (Table 3-1/3-2 notes, §4.1.3.2.4, Table 4-3):

1. **PPS selection** — pick the PPS APDO whose range reaches 20 V (min ≤ 9 V) with the highest current;
   otherwise one that reaches 15 V.
2. **Fixed PDOs** — passed through from the charger, ≤ 20 V, each capped at 3 A.
3. **AVS is offered only if the charger has a 15 V Fixed PDO** (spec requirement) and a PPS was selected.
4. **AVS current** — 9–15 V uses the 15 V Fixed current, 15–20 V uses the 20 V Fixed current, where each
   is `min(charger Fixed current, PPS current, 3 A)`. The same value is written into the 15 V / 20 V Fixed PDOs.
5. **If the selected PPS tops out below 20 V**, the 20 V Fixed PDO is dropped and AVS goes to 15 V only.
6. **Requests** — a Fixed request maps to the charger's Fixed PDO; an AVS request (100 mV steps) maps to
   a PPS request at the same voltage with the PPS maximum current. Voltage changes are requested in one step.
7. **Sequence** — device Request → Accept → front Request → charger PS_RDY → front VBUS within ±5 % for
   3 consecutive samples → PS_RDY to device. PPS keep-alive every 5 s.
8. **Charger re-advertises (DPS)** — keep the device's voltage if still available and re-advertise;
   otherwise go back to 5 V and Hard Reset the device.
9. **No PPS** → Fixed only. **Non-PD charger** → 5 V only.

Protection: front VBUS outside ±5 % (3 samples) or current > 3.5 A for 50 ms → switch off and Hard Reset.

## Risks

- **Charge pump** — TIM2 set up from datasheet DS2 §4 steps; whether CCER channels 3/4 must be enabled
  is not documented. Check HVCP ≈ VHV + 8.5 V first.
- **SDK bug** — `PWR_CTLR_ISINKEN` is defined as `0x0100` in the SDK header; the reference manual, SVD and
  the SDK's own `ch32m030_pwr.c` use bit 10. USB PD needs ISINKEN, so the code uses bit 10.
- **ADC reference** is VDD33 (3.1–3.5 V spec). The ±5 % window may trip falsely; tune `ADC_VREF_MV`.
- **No output VBUS sensing** and no active discharge (HVOD2/DSCG not connected).
- **Gate turn-off** — Vgs can briefly approach −VBUS when the gate is pulled low.
- **AVS small steps** must finish within 50 ms; this relies on the charger's PPS response time.
- **PD2.0 devices** also receive the AVS APDO in the first Source_Capabilities.

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