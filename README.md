# BMCU Firmware – Calibration and Compatibility Notes

This BMCU firmware has been tested and verified with the latest Bambu Lab A1 firmware v1.07.02.

IMPORTANT:
The printer must be configured as AMS, not AMS Lite.
Using AMS Lite will cause incompatibility issues.

# IMPORTANT – HMS WARNING STATUS

This firmware version **triggers an HMS warning immediately after printer startup**.

Important clarification:
- This HMS warning **does NOT block BMCU operation**
- It does **NOT require restarting the printer**
- It does **NOT affect printing**
- The printer works normally despite the warning
- The issue is **purely visual / informational** (HMS icon only)

At the moment, the HMS warning is known and accepted behavior in this firmware version.

---

## Supported printers

- Bambu Lab A1
- Bambu Lab A1 mini
- Bambu Lab P1S

Other printers may also work, but they have not been tested.

---

## Download

Please download ready-to-use firmware from the **"Releases"** section (right side of the GitHub page).
All firmware variants are generated there together with **.txt guides** that explain which build you should choose.

---

## Folder structure (generated firmware)

Firmware is generated into this tree:

firmwares/
- AUTOLOAD/
    - FILAMENT_RGB_ON/
        - SOLO/
        - AMS_A/
        - AMS_B/
        - AMS_C/
        - AMS_D/
    - FILAMENT_RGB_OFF/
        - SOLO/
        - AMS_A/
        - AMS_B/
        - AMS_C/
        - AMS_D/
- NO_AUTOLOAD/
    - FILAMENT_RGB_ON/
        - SOLO/
        - AMS_A/
        - AMS_B/
        - AMS_C/
        - AMS_D/
    - FILAMENT_RGB_OFF/
        - SOLO/
        - AMS_A/
        - AMS_B/
        - AMS_C/
        - AMS_D/

---

## SOLO firmware

Example file:

solo_0.095f.bin

This firmware is intended for single BMCU (SOLO) operation.

- Recommended for single-BMCU setups
- Filament retraction length: 9.5 cm

---

## Filament retraction explanation

Filament retraction must be calculated from the end of the AMS splitter inside the printer
(the plastic AMS part where four PTFE tubes enter).

Example:

- Distance from BMCU to the end of the AMS splitter: approximately 9.0 cm
- SOLO firmware retracts the filament about 0.5 cm past the splitter
- Total retraction length: 9.5 cm

When calculating your own retraction length:

- Always measure from the end of the AMS splitter
- Add the required distance plus approximately 9 cm, depending on your setup

---

## AMS_A / AMS_B / AMS_C / AMS_D firmware

These firmware versions are intended for:

- Multi-BMCU setups
- Longer filament retraction distances

If you want to use SOLO mode with a longer retraction, use AMS_A instead of SOLO.

---

## Calibration (first start)

Correct calibration is mandatory.
Without proper calibration, BMCU will not work correctly.

The calibration process is shown in the following video:

https://www.youtube.com/shorts/Hn_DNzSmhuc

Follow the calibration steps shown in the video carefully.

---

## Re-calibration

You can recalibrate the BMCU at any time.

Steps:

1. Remove all filaments from all channels
2. Hold any one buffer in position for approximately 5 seconds

---

## Safety and usage notes

- Do not flash BMCU while it is connected to the printer
- Do not disconnect BMCU while the printer is powered on
- Do not update printer firmware while BMCU is connected

These recommendations are based on community reports.
Not all failure scenarios have been tested.

Changing the printer mode from AMS Lite to AMS while BMCU was connected did not cause issues in testing, but this is not recommended.

---

## Unsupported requests

Please do not ask:

- how to flash BMCU
- how to configure or use multi-BMCU setups

These topics are well documented online and will not be answered.

---

## Bug reports

If you encounter a real bug, you may report it.
This firmware has undergone solid testing, and no issues are expected.

---

## Tested configuration

- Printer: Bambu Lab A1 / A1 mini
- Printer firmware: v1.07.02
- Printer mode: AMS (not AMS Lite)

---

# Changelog

## V7

### User-visible changes
- **Remember loaded filaments (persistent state).**  
  You can load filament and safely power off the printer.  
  This allows you to disable the automatic unload-at-end behavior in G-code (if you often print with one filament),
  keeping filament loaded until you actually need to change it.  
  More info here: https://wiki.bambulab.com/en/ams/manual/ams-not-unloading-to-save-filament
- **100% solved filament loading problems.** The system is stable and consistent across hardware variants.
- **Filament RGB colors.** Modules/LEDs can display the configured filament color.

### AUTOLOAD (short)
**How AUTOLOAD works**
- **DM (two microswitches):**
    - Touch first switch → AUTOLOAD starts (you may need a light manual push until gears grab).
    - BMCU feeds filament until the second switch (behind extruder) confirms **fully inserted**.
    - Then it feeds 120 mm to make it print-ready.
    - **Anti-snag protection:** buffer position is monitored; if the filament catches on housing / PTFE edge, it retracts to safe position and retries (3 retries).
- **Single-switch boards:**
    - Stage 1 is manual (no second switch to confirm fully-in).
    - Once filament is fully in the extruder, Stage 2 behaves the same (incl. anti-snag protection).

### Technical changes
- **ADC_DMA upgraded (ADC1 + ADC2 in parallel):**
    - Regular simultaneous mode: ADC1+ADC2 scan channels in parallel to reduce noise and increase throughput.
    - Lower noise enabled smaller filtering and faster stable readout.
    - Full filtered update time: **~5 ms instead of ~28 ms**.
- **AS5600 reading correctness improved** (robustness and stability of reads).
- **Timer/tick safety (wrap safety):** all time comparisons reviewed to be correct under wrap-around.
- Final stabilization and cleanup: overall behavior is faster and more deterministic than previous releases.
- There were more fixes in V7 as well; easiest is to check the commit history.

**Final note:** all known issues were ultimately resolved. BMCU is fully stable and significantly faster vs older firmware.
At this moment I do not expect any further fixes.

---

## V6

## Framework
- Dropped Arduino Core (PlatformIO: framework = arduino) - the whole firmware was rewritten to pure CH32 (WCH SDK / noneos).
- Direct use of hardware timers, DMA and interrupts - no Arduino delays, no random timing, deterministic real-time behavior.
- Faster and correct flash operations (WCH Fast API) - stable writes, faster, without corrupting neighboring data.

## ADC_DMA
- Separated DMA writes from CPU reads - previously reads happened while DMA was overwriting the buffer.
- Filter is computed in the background (DMA half/full), not during readout - previously `get_value()` blocked CPU and broke timing.
- Constant CPU load - previously larger filter window slowed the system down.
- DMA error handling

## BUS (BambuBus + AHUB)
- Fixed RX/TX buffer race (reading and overwriting the same buffer at the same time).
- Snapshot-based parsing instead of working on a live buffer
- Deterministic frame handling timing - constant CPU cost, independent from packet length.
- Robustness against transmission errors - a bad packet does not break the whole system state.

## Flash / NVM
- Flash written page-by-page (256B) instead of erasing/programming the whole sector (4KB)
- Write only when data actually changed
- Hardware CRC for flash + verification on read
- AMS data split into separate records - changing one filament does not rewrite the whole structure.

## Soft-I2C / AS5600
- Rewritten from Arduino, removed timing bugs and Arduino "magic".
- Correct ACK/NACK, START/STOP, recovery handling
- Hard isolation of channels with errors

## Motion / mechanics
- Smoother motor control
- Added calibration buffers - filament stays in a neutral position, without unnecessary tension.
- Better state transitions - no jerks and no sudden braking.

## Misc
- CRC8 / CRC16 rewritten to simple C + lookup tables - faster, deterministic, no objects and no runtime init.
- Partially de-spaghettified includes
- and many more - firmware prepared for further development


## Final note

This firmware started as a personal CH32 learning project.
During development it grew far beyond the original scope because working on BMCU turned out to be genuinely enjoyable.

Many solutions are intentionally overengineered.
Everything was implemented primarily for personal use and experimentation.

The firmware has been used extensively during development,
and no practical issues were observed in real-world usage.
