# Gearotons M17 — "Premium Minimal" derivative variations

Tom picked variant **"10 · Premium Minimal"** (`design_variations/v10.html`, copied here as
`_BASELINE_v10_premium_minimal.html`) as his favourite of 40 designs. This round produces **20 new
variations that all descend from that one design.** Read the baseline file before you write anything.

You are building **ONE** standalone, self-contained `.html` file. Everything (CSS + any JS) inline.
No external fonts, CDNs, frameworks, or network requests of any kind.

---

## 1. The Premium Minimal DNA — keep this

The baseline's character, which every variation must still read as:

- **Apple-keynote restraint.** Huge amounts of whitespace (~130px section padding on desktop),
  a narrow content column (~1080px max), nothing shouting.
- **Light-weight display type at large sizes.** `font-weight:300` headlines at `clamp(34px..64px)`
  with a `<strong>` second line at weight 600. Tight negative letter-spacing (-.02 → -.03em).
- **Two-tone headline pattern**: a light-weight first line, a bold second line — e.g.
  "Motion control. / **All in one.**" This is a signature. You may evolve it, not abandon it.
- **Grey supporting copy** (`--grey:#6e6e73`) under near-black ink (`--ink:#1d1d1f`).
- **Hairline rules** (`1px solid #e8e8ed`) instead of boxes and borders everywhere.
- **Pill buttons** (`border-radius:980px`) with a thin outline that fills with the brand green on
  hover, plus an "arrow link" whose arrow slides right on hover.
- **The green is a seasoning, not a sauce** — used for the accent keyline, hover fills, numerals,
  small marks. Never large flat green areas (unless your assigned direction explicitly says so).
- **One dark section** (near-black `#0b0b0c`) hosting the code sample, as a rhythm break.
- **Sticky translucent nav** with backdrop blur, 56px tall.
- **Uppercase letterspaced eyebrows** (13px, .14em) above section headlines.
- **Products float** — transparent-cutout PNGs on the page background, no photo frames.

**Every variation must still feel premium, calm, expensive, and confident.** If your assigned
direction pulls toward density or decoration, apply it with restraint: this is a *variation on*
Premium Minimal, not a replacement for it.

---

## 2. What to VARY

Your assigned direction (given separately) tells you what to change. Legitimate axes:

typography (family stacks, scale, weight contrast, serif vs grotesk vs mono) · palette
(warm/cool neutrals, ivory, graphite, inverted dark) · accent colour usage · layout rhythm
(centered vs left-aligned vs asymmetric split) · grid and column structure · section transitions
and banding · the shape language (radii, rules, cards) · density · hover/scroll behaviour ·
how the three-model comparison is presented · how the spec data is organised · imagery scale
and cropping · **and the marketing copy itself.**

**Copy is fair game and should be improved.** Rewrite headlines, eyebrows, feature titles,
section intros and CTAs to be more persuasive, more specific, and better matched to your visual
direction. Keep every *fact* exactly as stated in §4. Do not invent prices, awards, customer
counts, testimonials, benchmarks, certifications, or claims that aren't below.

---

## 3. Required sections (all 20 variations)

The baseline v10 dropped a lot of real product information to stay short. **These variations must
put it back** — Tom explicitly asked that all important information from the marketing page be used.
Arrange and name the sections however your direction dictates, but the following must all be present
and legible:

1. **Nav** — logo + wordmark, a few anchors, a "Shop" CTA button.
2. **Hero** — product name, positioning line, primary CTA (Shop) + secondary (Documentation),
   hero product image, and the headline specs (NEMA 17 mount · 12–24 V · closed loop · 32 kHz PID).
3. **What it is / integration story** — motor + driver + motion controller + encoder in one package.
4. **One bus, many motors** — RS-485 daisy-chain, single connection point, plus the
   **connection diagram** image.
5. **Key features** — the substance of the 16 features in §4.3 (you may merge/re-title them, but
   don't silently drop capabilities).
6. **The three models** — M17-60 / M17-48 / M17-40 with the **full** spec comparison (§4.4 and
   §4.5 — mechanical AND electrical, including shaft length, shaft diameter and protection class)
   and the three dimension drawings.
7. **Operating conditions** — the four rows in §4.6.
8. **Getting started / developers** — the Python example, the Arduino example, `pip3 install
   servomotor`, the Arduino Library Manager name, and the getting-started guide with its five
   bullets and its link.
9. **Unit system** — the libraries convert units for you; show the supported units (§4.9).
10. **Communication protocol** — RS-485 facts in §4.10, including baud rate, addressing,
    firmware update over the bus, the command groups, and the `servomotor_command -c` /
    `motor_commands.json` pointers.
11. **LEDs & buttons** — §4.11, with the `motor_back` image.
12. **What's in the box / the adapter** — §4.12, with the kit and/or adapter image.
13. **Applications** — the three application photos + the wider list in §4.13.
14. **Open source** — the two logos, the GitHub link, the belief statement.
15. **Company** — the startup story in §4.14.
16. **Footer CTA + footer** — final Shop CTA, links, copyright, and the registered company name
    and address from §4.15.

Sections 9, 10, 11, 12 are the dry ones — this is where your design has to work hardest. Handle
them with premium-minimal restraint (compact hairline grids, small-caps labels, two-column
definition lists, tidy tables). Do not let the page decay into a datasheet wall. It is fine to
use `<details>`/`<summary>` disclosure for the driest reference material *if* you style it well.

---

## 4. The facts — verbatim source of truth

### 4.1 Brand
- Company: **Gearotons** — trading name. Registered entity: **Green Eco Technology (Shenzhen)
  Company Limited**, Room C301E, 3F, Block CD, Tianjing Building, Shatou Street, Futian District,
  Shenzhen City, Guangdong Province, China.
- Product: **M17 Series Servomotors**.
- Taglines currently in use: *"Affordable and Simple All-in-One Motion Control"* and
  *"From Education to Innovation"*. You may write better ones — keep the meaning honest.
- Brand green ≈ `#7AB648` (from the logo). Black + white are the other brand colours.
- Site: https://gearotons.com · Store: https://gearotons.com/store ·
  Tutorial/getting-started: https://tutorial.gearotons.com ·
  GitHub: https://github.com/tomrodinger/servomotor

### 4.2 What it is (intro, may be rephrased — keep the facts)
The M17 Series Servomotors are all-in-one motion control solutions that integrate a motor, motor
driver, motion controller, and encoder in a single compact package. They use an RS-485
communication interface, so multiple units can be daisy-chained and controlled from a single
connection point. Three models — M17-60, M17-48, M17-40 — offer flexible torque options while
keeping identical control characteristics.

Each motor has sophisticated control features: multiple operation modes, self-calibration, and
built-in status monitoring through LED indicators. They can be controlled from any platform —
Mac, PC, Raspberry Pi, Arduino, ESP32 — requiring only a low-cost RS485 adapter, which makes them
suitable for both educational and industrial use.

The series supports precise position control with trapezoid movement profiles, closed-loop control
mode, and comprehensive error handling. With standard NEMA 17 mounting dimensions, a wide voltage
range (12–24 V), and a robust communication protocol, they are a reliable, flexible solution
wherever precise motion is needed — robotics, CNC machines, automated test equipment, scientific
instruments.

### 4.3 Key features (all 16 — merge/re-title freely, don't lose capabilities)
1. High level of integration: motor + motor driver + motion control system + encoder in one unit.
2. Control any number of motors from one simple controller over a single RS-485 interface.
3. High-level commands such as "Enable MOSFETs" and "Trapezoid move" — no timing-critical
   DIR/STEP pulse generation.
4. Compact form factor, nearly the same size as a NEMA 17 stepper of the same specification,
   with no protrusions.
5. Standardised NEMA 17 mounting dimensions.
6. Wide voltage range, 12–24 V, for flexible power options.
7. High-precision closed-loop control with a built-in encoder and a PID control loop running at
   **32 kHz**.
8. Much more power efficient in closed-loop control than a conventional stepper.
9. Integrated over-current, over-voltage and over-temperature protection.
10. Maximum speed reaches **560 RPM**.
11. Torque-to-weight ratio is the same as an equivalent stepper motor.
12. Compatible with Raspberry Pi, Arduino, ESP32, Mac and PC.
13. Excellent documentation and tutorials to get running fast.
14. **AI-friendly documentation** — point your favourite AI assistant at it and let it do the work.
15. Suitable for robotics, CNC, automation, scientific instruments, testing jigs, 3D printers,
    and more.
16. Available in three sizes, so you can pick the right torque and price for the application.

### 4.4 Mechanical specifications
| Parameter | M17-60 | M17-48 | M17-40 |
|---|---|---|---|
| Dimensions (L×W) | 42.2 × 42.2 mm | 42.2 × 42.2 mm | 42.2 × 42.2 mm |
| Height | 59.8 mm | 48.6 mm | 41.6 mm |
| Shaft length | 20.4 mm | 20.4 mm | 18.5 mm |
| Shaft diameter | 5 mm | 5 mm | 5 mm |
| Weight | 470 g | 360 g | 285 g |
| Protection class | IP20 | IP20 | IP20 |

### 4.5 Electrical specifications
| Parameter | M17-60 | M17-48 | M17-40 |
|---|---|---|---|
| Operating voltage | 12–24 V | 12–24 V | 12–24 V |
| Rated torque | 0.65 N·m | 0.55 N·m | 0.42 N·m |
| Maximum speed | 560 RPM | 560 RPM | 560 RPM |
| Maximum current | 1.1 A | 1.1 A | 1.1 A |
| Rated power | 38 W | 32 W | 25 W |

### 4.6 Operating conditions
| Parameter | Specification |
|---|---|
| Operating temperature | 0 °C to +80 °C |
| Storage temperature | −20 °C to +60 °C |
| Humidity range | 20 % to 80 % RH (non-condensing) |
| Installation environment | Indoor use only |

### 4.7 Python example (use verbatim, or the shorter form below it)
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
Install: `pip3 install servomotor`

### 4.8 Arduino example (condensed from the real example — keep it short and correct)
```cpp
#include <Servomotor.h>

Servomotor motor('X', Serial1);      // alias 'X' on the RS485 UART

void setup() {
  motor.setPositionUnit(PositionUnit::SHAFT_ROTATIONS);
  motor.setTimeUnit(TimeUnit::SECONDS);

  motor.systemReset();
  delay(1500);                       // let the bootloader window pass
  motor.enableMosfets();
  motor.trapezoidMove(1.0, 1.0);     // one rotation in one second
}

void loop() {}
```
Available in the Arduino IDE Library Manager as **"Servomotor"**.

### 4.9 Unit system
The motors use integer internal units so motion maths stays fast. The Python and Arduino libraries
convert automatically, so you work in whatever unit you prefer:

| Quantity | Available units |
|---|---|
| Time | seconds, milliseconds, minutes, microseconds, timesteps |
| Position | shaft rotations, degrees, radians, encoder counts |
| Velocity | rotations per second, RPM, degrees per second, radians per second, counts per second, counts per timestep |
| Acceleration | rotations per second², RPM per second, degrees per second², radians per second², counts per second², counts per timestep² |
| Current | internal current units, milliamps, amps |
| Voltage | volts, millivolts |
| Temperature | celsius, fahrenheit, kelvin |

(You may show a subset per row if space demands, but keep all seven quantities.)

### 4.10 Communication protocol
- RS-485, **baud rate fixed at 230400**.
- Any number of motors daisy-chained on one bus.
- Every motor carries a **factory-assigned 64-bit unique address** so it can always be addressed
  individually.
- Each motor can also be given a **1-byte alias (0–251)** to keep packets short while still
  addressing it individually. **255 is the broadcast alias.**
- A comprehensive command set covers motion control, configuration and status monitoring.
- **Firmware update over the RS-485 interface is supported.**
- Commands are grouped: **Basic Control · Motion Control · Configuration · Status & Monitoring ·
  Device Management**.
- The living source of truth is `motor_commands.json`:
  https://github.com/tomrodinger/servomotor/blob/main/python_programs/servomotor/motor_commands.json
- Or print the whole command reference locally:
  ```bash
  pip3 install servomotor   # once
  servomotor_command -c
  ```

### 4.11 Indicator LEDs and buttons
- Two status LEDs, **green** and **red**. The green LED flashes slowly as a heartbeat, and quickly
  to indicate the bootloader is running rather than the application. The red LED lights briefly to
  show bus traffic, and flashes a certain number of times to indicate fatal error codes.
- Two buttons, **Reset** and **Test**. Reset restarts the microcontroller and returns all state to
  defaults. Test spins the motor: a brief press spins one way; press for more than 0.3 s and
  release to spin the other way; hold at least 2 s and release to enter closed-loop mode; hold
  more than 15 s and release to run a self-calibration. *The motor spins during calibration and
  must be able to turn freely, so remove any load first.*

### 4.12 What you need / what's in the box
- Requires a **low-cost RS485 adapter** (sold separately). Gearotons sells a 3-in-1 RS485 adapter.
- A cable is included with the motor.
- Works with Raspberry Pi, Arduino, ESP32, Mac and PC.

### 4.13 Applications
Robotics · CNC machines · Automated testing equipment · Scientific instruments · Industrial
automation · 3D printers · Education and teaching labs · Test jigs.

### 4.14 Company profile
An innovative startup committed to making precision motion control accessible to everyone —
makers, educators and engineers alike. Founded in Shenzhen in 2022 by a Canadian entrepreneur,
focused on integrated servo motor systems that combine cost-effectiveness, high integration, ease
of use and high performance. The M17 series is the first product. All software, firmware and PCB
design files are open source, on GitHub. "We believe in making the world better through
technology."

### 4.15 Legal / footer
© 2026 Gearotons · Green Eco Technology (Shenzhen) Company Limited · Room C301E, 3F, Block CD,
Tianjing Building, Shatou Street, Futian District, Shenzhen City, Guangdong Province, China.

---

## 5. Image assets (paths are relative to THIS folder — your HTML lives here)

Transparent cutouts (preferred for products — they float on any background):
- `../transparent/one_motor_transparent_small.png` — hero-quality 3/4 studio shot, single motor
- `../transparent/one_motor_transparent.png` — full-res version
- `../transparent/M17_series_overview_transparent_small.png` — the three motors side by side
- `../transparent/M17_series_overview_transparent.png` — full-res version
- `../transparent/kit_with_three_motors_transparent_small.png` — kit flat-lay: 3 motors, adapter
  PCB, cables, knobs
- `../transparent/motor_back_transparent_small.png` — motor rear: spec label, QR codes, Reset/Test
  buttons, LEDs
- `../transparent/adapter_and_wire_transparent_small.png` — the green 3-in-1 RS485 adapter + cable

Photographs with real backgrounds (do NOT expect transparency):
- `../robotics.jpg` — motor in a CNC/robotics metal assembly with a large gear (squarish)
- `../automation.jpg` — motor mounted in an automation machine
- `../test_rack.jpg` — motors in an automated test rack
- `../connection_diagram.jpg` — wiring diagram, white background, technical (landscape, wide)

Technical drawings, white background:
- `../preview/public/marketing/images/M17-60_dimensions.png`
- `../preview/public/marketing/images/M17-48_dimensions.png`
- `../preview/public/marketing/images/M17-40_dimensions.png`

Logos:
- `../Gearotons_Logo.png` — logo mark, transparent background (black gear + green arms)
- `../Gearotons_Logo_and_Gearotons_Name.png` — logo + wordmark, **white background** (only safe on
  white/very light)
- `../Open_Source_Initiative.svg.png`, `../Open-source-hardware-logo.svg.png` — open-source logos,
  white/transparent background

Notes: the dimension drawings and the connection diagram are dark line art on white. On a dark
theme, put them in a light card, or invert them with `filter: invert(1) hue-rotate(180deg)` —
whichever you can make look deliberate.

---

## 6. Hard requirements

- One file, fully self-contained. No `@import`, no `<link>` to a font service, no CDN scripts.
  System/web-safe font stacks only (`-apple-system`, `"SF Pro Display"`, `Helvetica Neue`,
  `Georgia`, `Iowan Old Style`, `Palatino`, `Avenir`, `Futura`, `Optima`, `Charter`,
  `ui-monospace`, `Menlo`, `SFMono-Regular`, etc.).
- Fully responsive, 360px → 1920px. `img{max-width:100%;height:auto;display:block;}`.
  **No horizontal page scroll at any width** — wide tables must scroll inside their own
  `overflow-x:auto` container.
- Real hover states on every button and link. Focus-visible outlines are welcome.
- Consistent spacing scale. Generous whitespace. Real typographic hierarchy.
- `<title>` = `Gearotons — M17 Series Servomotors` (you may append your direction name after an
  em dash if you like).
- Shop CTAs → `https://gearotons.com/store`. Docs → `https://gearotons.com`.
  Guide → `https://tutorial.gearotons.com`. GitHub → `https://github.com/tomrodinger/servomotor`.
- Any JS must be tiny, inline, and degrade gracefully (e.g. an IntersectionObserver reveal that
  starts from a visible state if JS is off — never leave content hidden).
- Honest: no invented prices, awards, customer counts, testimonials, or performance claims.
