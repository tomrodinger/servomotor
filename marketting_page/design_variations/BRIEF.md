# Gearotons M17 Marketing Page — Shared Content Brief

You are building ONE standalone marketing page variant (a single self-contained .html file).
This brief contains ALL the factual content. Do not invent specs, prices, or claims not listed here.

## Company / brand
- Company: **Gearotons** (young startup, founded in Shenzhen 2022 by a Canadian entrepreneur)
- Product: **M17 Series Servomotors** — all-in-one motion control
- Tagline currently used: "Affordable and Simple All-in-One Motion Control"
- Brand colors: green (≈ #7AB648, from the logo), black, white. (Variants may propose other palettes if the design direction says so.)
- Live site: https://gearotons.com — CTA buttons should link to https://gearotons.com/store
- Logo images (black gear + green robot arms, transparent/white background):
  - `../Gearotons_Logo.png` (logo mark only)
  - `../Gearotons_Logo_and_Gearotons_Name.png` (logo + wordmark, white background — only good on white)

## Available images (paths are relative to the design_variations/ directory where your HTML file lives)
All product photos are BLACK motors on WHITE/light backgrounds — on dark themes place them in light cards or accept the white background creatively.
- `../M17_series_overview.jpg` — the three motors side by side with model labels (good hero image, landscape)
- `../one_motor.jpg` — beautiful 3/4 studio shot of a single motor, white bg (large, portrait-ish)
- `../kit_with_three_motors.jpg` — full kit flat-lay: 3 motors, adapter PCB, cables, knobs (portrait)
- `../motor_back.jpg` — back of motor showing printed specs label + QR codes, Reset/Test buttons (portrait)
- `../adapter_and_wire.jpg` — green "3-in-1 RS485 Adapter" PCB with cable (landscape)
- `../automation.jpg` — motor mounted in an automation machine (application photo)
- `../robotics.jpg` — motor in a CNC/robotics metal assembly with large gear (application photo, square)
- `../test_rack.jpg` — motors in a testing rack (application photo)
- `../connection_diagram.jpg` — wiring diagram (white background, technical)
- `../preview/public/marketing/images/M17-40_dimensions.png`, `../preview/public/marketing/images/M17-48_dimensions.png`, `../preview/public/marketing/images/M17-60_dimensions.png` — CAD dimension drawings (white bg)
- `../Open_Source_Initiative.svg.png`, `../Open-source-hardware-logo.svg.png` — open source logos

## Intro copy (may be shortened/rephrased for marketing punch, keep facts)
The M17 Series Servomotors are all-in-one motion control solutions that integrate a motor, motor driver, motion controller, and encoder in a single compact package. RS-485 interface lets you daisy-chain any number of motors and control them from a single connection point. Three models — M17-60, M17-48, M17-40 — offer flexible torque options with consistent control characteristics.

Control from any platform: Mac, PC, Raspberry Pi, Arduino, ESP32 (needs only a low-cost RS485 adapter). Sophisticated control: multiple operation modes, self-calibration, LED status indicators, trapezoid movement profiles, closed-loop control, comprehensive error handling. Standard NEMA 17 mounting, 12–24V, robust protocol. Ideal for education AND industry: robotics, CNC, automated testing, scientific instruments.

## Key features (pick the best ones; you don't need all 16)
1. Motor + driver + motion controller + encoder integrated in one package
2. Control any number of motors over one RS-485 bus from one controller
3. High-level commands like "Enable MOSFETs" and "Trapezoid move" — no timing-critical STEP/DIR signals
4. Nearly the same size as a NEMA 17 stepper with the same specs (no protrusions)
5. Standard NEMA 17 mounting dimensions
6. Wide voltage range 12–24V
7. High-precision closed-loop control: built-in encoder + PID loop running at 32 kHz
8. Much more power efficient in closed loop vs. a conventional stepper
9. Integrated over-current, over-voltage, over-temperature protection
10. Max speed 560 RPM
11. Same torque-to-weight ratio as an equivalent stepper
12. Works with Raspberry Pi, Arduino, ESP32, Mac, PC
13. Excellent documentation and tutorials
14. AI-friendly documentation (let your favorite AI do the work)
15. Suitable for robotics, CNC, automation, scientific instruments, test jigs, 3D printers
16. Three sizes to match torque and budget

## Technical specifications (table)
| Parameter | M17-60 | M17-48 | M17-40 |
|---|---|---|---|
| Operating Voltage | 12–24V | 12–24V | 12–24V |
| Rated Torque | 0.65 N·m | 0.55 N·m | 0.42 N·m |
| Maximum Speed | 560 RPM | 560 RPM | 560 RPM |
| Maximum Current | 1.1 A | 1.1 A | 1.1 A |
| Rated Power | 38 W | 32 W | 25 W |
| Height | 59.8 mm | 48.6 mm | 41.6 mm |
| Footprint | 42.2 × 42.2 mm | 42.2 × 42.2 mm | 42.2 × 42.2 mm |
| Weight | 470 g | 360 g | 285 g |

## Python code example (for developer-oriented sections; Arduino also supported)
```python
import servomotor
from servomotor import M3

servomotor.open_serial_port()
motor = M3("X", time_unit="seconds",
                position_unit="degrees")
motor.system_reset()
motor.enable_mosfets()
motor.trapezoid_move(360, 2)  # one revolution in 2 s
```
Install: `pip3 install servomotor` · Arduino library available in the Library Manager ("Servomotor")

## Applications (with matching photos)
Robotics · CNC machines · Automated testing equipment · Scientific instruments · Automation · 3D printers · Education

## Company profile (short)
An innovative startup making precision motion control accessible to everyone — makers, educators, and engineers. Founded in Shenzhen in 2022 by a Canadian entrepreneur. Open source hardware and software: firmware, libraries, and schematics on GitHub (https://github.com/tomrodinger/servomotor).

## Page requirements (every variant)
- ONE self-contained HTML file: all CSS (and any JS) inline. NO external fonts/CDNs/frameworks. System font stacks or web-safe fonts only (you may use distinctive stacks like Georgia, Avenir, Futura, Menlo, ui-monospace etc.).
- Sections to include (order/arrangement up to your design direction): sticky or top nav with logo + "Shop" CTA · hero · key features · the three models w/ spec comparison · how easy it is to get started (code sample) · applications · open source + company blurb · footer with CTA.
- Fully responsive (mobile → desktop). Images `max-width:100%`. No horizontal page scroll.
- Professional polish: consistent spacing scale, real visual hierarchy, hover states on buttons/links, generous whitespace. This page must look like it could sell a lot of servomotors.
- Buttons: "Shop Now" → https://gearotons.com/store ; "Documentation" can link to https://gearotons.com
- Put a `<title>` like "Gearotons — M17 Series Servomotors".
- Keep it honest: no invented prices, awards, customer counts, or testimonials.
