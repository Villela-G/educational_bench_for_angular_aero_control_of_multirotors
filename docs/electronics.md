---
layout: docs
title: Electronics
---

# Electronics

## Overview

The electronic system is composed of modular PCBs responsible for power distribution, sensing, and data acquisition. All boards were designed using open-source tools and are available for modification and reproduction.

**Important:**  
All components must be assembled according to the provided PCB layout files (`.brd` and `.kicad_pcb`). Component placement, orientation, and routing should strictly follow the design files.

---

## Power Supply Board

This board is responsible for distributing power to the system and interfacing with the external power supply.

**Main components:**
- 3300 µF capacitor  
- 22 nF capacitor  
- KCD6 switch  
- 2 × KRE 2V connectors  
- Banana connectors (male and female, red and black)  
- XT60 connectors (male and female)  

---

## Weather Station Board

This module provides environmental sensing and auxiliary measurements.

**Main components:**
- ESP32 Wi-Fi module  
- DHT22 temperature and humidity sensor  
- BMP280 pressure sensor  
- L7805 voltage regulator + heat sink  
- 10 kΩ resistor  
- 22 nF capacitor  
- 2 × 102 nF capacitors  
- 2 × current sensors  
- KRE and KRV connectors  
- Switch  

---

## Hall Sensor Boards (×2)

These boards are used for angular velocity measurement.

**Main components:**
- 4 × DRV5011 Hall sensors  
- 4 × 22 nF capacitors  
- 4 × KRE 2V connectors  

---

## Potentiometer Board

This board provides angular position measurement and signal conditioning.

**Main components:**
- Precision potentiometer  
- Standard potentiometer  
- L7808 voltage regulator + heat sink  
- 2 × LM358 operational amplifiers  
- 2 × 1 µF capacitors  
- 4 × 100 nF capacitors  
- 2 × 33 kΩ resistors  
- 2 × 10 kΩ resistors  
- KRE 2V and 3V connectors  

---

## Wiring and Integration Guidelines

- Female connectors should be soldered directly onto the PCBs, while male connectors are used on cables  
- Banana connectors are used for motor power connections  
- The XT60 connector interfaces the external power supply with the system  
- Power lines should use silicone-insulated wires  
- Sensor connections should use shielded cables to reduce noise  
- PCBs can be mounted on the aluminum structure using hammer nuts and screws  
- The potentiometer board must be mechanically coupled to the shaft for accurate angle measurement  
- Wiring layout may be adapted depending on system configuration and physical constraints  

---

<div style="display: flex; justify-content: space-between; margin-top: 60px;">
  <a href="hardware.html">← Previous: Hardware Design</a>
  <a href="bom.html">Next: Bill of Materials →</a>
</div>