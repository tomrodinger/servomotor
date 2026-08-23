# Shared facts for Variation C. No invented claims.

SHOP = "https://gearotons.com/store"
DOCS = "https://gearotons.com"
GITHUB = "https://github.com/tomrodinger/servomotor"

TITLE = "Gearotons — M17 Series Servomotors"
TAGLINE = "Affordable and Simple All-in-One Motion Control"

CODE = """import servomotor
from servomotor import M3

servomotor.open_serial_port()
motor = M3("X", time_unit="seconds",
                position_unit="degrees")
motor.system_reset()
motor.enable_mosfets()
motor.trapezoid_move(360, 2)  # one revolution in 2 s"""

INSTALL = 'pip3 install servomotor'
ARDUINO = 'Arduino library: Library Manager → “Servomotor”'

SPECS = [
    ("Operating Voltage", "12–24V", "12–24V", "12–24V"),
    ("Rated Torque", "0.65 N·m", "0.55 N·m", "0.42 N·m"),
    ("Maximum Speed", "560 RPM", "560 RPM", "560 RPM"),
    ("Maximum Current", "1.1 A", "1.1 A", "1.1 A"),
    ("Rated Power", "38 W", "32 W", "25 W"),
    ("Height", "59.8 mm", "48.6 mm", "41.6 mm"),
    ("Footprint", "42.2 × 42.2 mm", "42.2 × 42.2 mm", "42.2 × 42.2 mm"),
    ("Weight", "470 g", "360 g", "285 g"),
]

MODELS = [
    {"name": "M17-60", "torque": "0.65 N·m", "power": "38 W", "height": "59.8 mm", "weight": "470 g", "dim": "../M17-60_dimensions.png"},
    {"name": "M17-48", "torque": "0.55 N·m", "power": "32 W", "height": "48.6 mm", "weight": "360 g", "dim": "../M17-48_dimensions.png"},
    {"name": "M17-40", "torque": "0.42 N·m", "power": "25 W", "height": "41.6 mm", "weight": "285 g", "dim": "../M17-40_dimensions.png"},
]

FEATURES = [
    ("All-in-one package", "Motor, driver, motion controller, and encoder in one compact body."),
    ("One bus, any number", "Daisy-chain motors on a single RS-485 connection."),
    ("High-level commands", "“Enable MOSFETs” and “Trapezoid move” — no timing-critical STEP/DIR."),
    ("NEMA 17 size", "Nearly the same size as a stepper with the same specs. No protrusions."),
    ("Standard mounting", "Standard NEMA 17 hole pattern and footprint."),
    ("12–24 V", "Wide voltage range for bench supplies and industrial rails."),
    ("32 kHz closed loop", "Built-in encoder and a PID loop running at 32 kHz."),
    ("Efficient vs. stepper", "Closed loop draws what the load needs — not a constant holding current."),
    ("Onboard protection", "Over-current, over-voltage, and over-temperature."),
    ("560 RPM", "Maximum speed, all three models."),
    ("Stepper torque density", "Same torque-to-weight ratio as an equivalent stepper."),
    ("Any host", "Raspberry Pi, Arduino, ESP32, Mac, and PC."),
    ("Real documentation", "Tutorials and references written to get you moving."),
    ("AI-friendly docs", "Let your favorite AI read the manuals and write the first sketch."),
    ("Built for the bench", "Robotics, CNC, automation, instruments, test jigs, 3D printers."),
    ("Three sizes", "Match torque and budget: M17-60, M17-48, M17-40."),
]

INTRO = (
    "The M17 Series Servomotors are all-in-one motion control: motor, driver, "
    "motion controller, and encoder in a single compact package. RS-485 lets you "
    "daisy-chain any number of motors from one connection. Three models — "
    "M17-60, M17-48, M17-40 — share the same control characteristics at three torques."
)

INTRO2 = (
    "Control from Mac, PC, Raspberry Pi, Arduino, or ESP32 with a low-cost RS-485 "
    "adapter. High-level commands, self-calibration, LED status, trapezoid profiles, "
    "closed-loop control, and comprehensive error handling. Standard NEMA 17 mounting, "
    "12–24 V, a robust protocol. Education and industry: robotics, CNC, automated "
    "testing, scientific instruments."
)

COMPANY = (
    "Gearotons is a young startup making precision motion control accessible to "
    "makers, educators, and engineers. Founded in Shenzhen in 2022 by a Canadian "
    "entrepreneur. Firmware, libraries, and schematics are open source on GitHub."
)

APPS = [
    ("Robotics", "../robotics_small.jpg"),
    ("CNC", "../robotics_small.jpg"),
    ("Automated testing", "../test_rack_small.jpg"),
    ("Scientific instruments", "../automation_small.jpg"),
    ("Automation", "../automation_small.jpg"),
    ("3D printers", "../transparent/kit_with_three_motors_transparent_small.png"),
    ("Education", "../transparent/M17_series_overview_transparent_small.png"),
]


def spec_rows():
    return "\n".join(
        f"<tr><th>{p}</th><td>{a}</td><td>{b}</td><td>{c}</td></tr>"
        for p, a, b, c in SPECS
    )


def esc(s):
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
    )


def shell(title, css, body, extra_head=""):
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
{extra_head}
<style>
{css}
</style>
</head>
<body>
{body}
</body>
</html>
"""
