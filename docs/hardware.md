---
layout: docs
title: Hardware Design
---

# Hardware Design

## Hardware in Context

The proposed system is an open-source experimental testbed designed to reproduce the angular dynamics of a multirotor aerial vehicle around a single rotational axis.

The platform consists of a rigid aluminium beam actuated by two opposed brushless motors, free to rotate about its center through low-friction bearings. By constraining motion to one degree of freedom (1-DOF), the system enables safe and controlled benchtop experiments while preserving the key challenges of sensing, actuation, and feedback control present in real aerial vehicles.

Single-axis testbeds are widely used in aerial robotics for education and early-stage validation. However, many existing implementations rely on custom-machined components or tightly integrated setups, which can limit reproducibility and accessibility.

In contrast, this platform is designed with:

- Standardized aluminium profiles  
- Off-the-shelf propulsion components  
- Fully open electronics and firmware  

This results in a system that is low-cost, reproducible, and easily adaptable for academic and research environments.

---

## Hardware Overview

The system is built around a modular aluminium structure supporting a freely rotating beam driven by two motor–propeller units.

Key characteristics include:

- Modular mechanical design  
- Adjustable geometry and mass distribution  
- Open embedded control (STM32-based)  
- External power supply for repeatable experiments  
- Integrated sensing for angle, motor rpm and environmental conditions  

The platform allows full access to low-level control, enabling experiments with different control strategies, communication protocols, and sensing configurations.

---

## Applications

This hardware supports a wide range of use cases:

- Validation of control strategies for aerial systems  
- Propulsion and actuator characterization  
- Sensor testing and calibration  
- Hardware-in-the-loop experimentation  
- Teaching control systems and flight dynamics  

The constrained motion improves safety and repeatability while still capturing relevant nonlinearities of real propulsion systems.

---

## Design Files

All mechanical and electronic design files are openly available:

👉 [Access design files on Zenodo](https://zenodo.org/records/18993888)

### Included files

**Mechanical (CAD):**
- Structural aluminium profile  
- Full system assembly  
- Motor model  
- Shaft and bearing components  

**Electronics (schematics):**
- Hall effect sensor  
- Potentiometer sensing circuit  
- Power supply  
- Weather station  

**PCBs:**
- Hall sensor board  
- Potentiometer board  
- Power supply board  
- Weather station board  

All files are provided in open formats and can be modified or extended as needed.

---

<div style="display: flex; justify-content: space-between; margin-top: 60px;">
  <a href="documentation.html">← Previous: Documentation</a>
  <a href="electronics.html">Next: Electronics →</a>
</div>