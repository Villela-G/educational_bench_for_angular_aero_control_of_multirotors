# Educational Bench for Angular Aero Control of Multirotors

This repository contains the code associated with an open-source educational test bench designed to study and validate attitude control strategies in multirotor systems.

The platform reproduces the angular dynamics of a drone around a single rotational axis (1-DOF), enabling safe, low-cost, and repeatable experiments using real propulsion hardware and embedded sensing.

🔗 **Full documentation:**
https://villela-g.github.io/educational_bench_for_angular_aero_control_of_multirotors/

---

## Code Overview

The repository is organized into two main components:

### STM32 Control Firmware

This firmware implements the control loop of the test bench, including:

* Sensor acquisition
* Signal processing
* Actuator control

It runs on an STM32-based platform and serves as a reference implementation for real-time control of the system. Depending on the specific hardware configuration, adjustments may be required.

---

### Sensor Validation Codes (Arduino)

Individual test codes are provided for each sensor used in the platform. These scripts allow independent verification of sensor functionality before full system integration.

The following sensors are covered:

* Hall effect sensor
* BMP280 (pressure)
* DHT22 (temperature and humidity)
* Potentiometer (angular position)

These codes are particularly useful for debugging, calibration, and educational purposes.

**Note:** These scripts are intended for validation only and are not optimized for real-time control or final deployment.

---

## Purpose

This repository supports the reproducibility of the hardware platform by providing accessible and modular code for both control and instrumentation. It is intended for students, researchers, and developers interested in:

* Control systems for aerial robotics
* System identification and actuator characterization
* Embedded systems and sensor integration
* Hardware-in-the-loop experimentation

---

## License

This project is part of an open-source hardware initiative. Please refer to the repository license for usage and distribution terms.
