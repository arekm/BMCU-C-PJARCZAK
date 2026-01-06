# BMCU Firmware – Calibration and Compatibility Notes

This BMCU firmware has been tested and verified with the latest Bambu Lab A1 firmware v1.07.01.

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

Other printers may also work, but they have not been tested.

---

## Folder structure

SOLO/
AMS_A/
AMS_B/
AMS_C/
AMS_D/

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
- Printer firmware: v1.07.01
- Printer mode: AMS (not AMS Lite)
