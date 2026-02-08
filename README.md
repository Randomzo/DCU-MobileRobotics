# DCU-MobileRobotics
Mobile Robotics Motor Control Project
Project Overview

This project develops an autonomous two-wheel mobile robot using an ESP32 microcontroller and DRV8835 motor driver. The system is designed to progress from basic motion control to full autonomous navigation using sensing, communication, and pathfinding algorithms.

The robot currently supports:
Motor control using PWM + phase control
Line following using IR sensors and PD control
WiFi server communication
Early-stage pathfinding integration
The long-term goal is reliable autonomous navigation between nodes using shortest-path routing.

Team Members:
Niall Moran
Theo Troev
Avyay Mynampati
Neasa O’Byrne Hadjami
Esiyemeh Idogho

System Architecture

Hardware
-ESP32 microcontroller
-DRV8835 motor driver
-DC motors + differential drive chassis
-Line follower IR sensor array
-Ultrasonic sensor
-Sharp distance sensor

Software
-Arduino / ESP32 framework
-PWM motor control
-PD line following control
-WiFi communication
-Pathfinding algorithms (in progress)

Features Implemented by Week

Week 1 — Hardware + Motion
Robot chassis assembly
Basic motor movement

Week 2 — Line Following
PD feedback control for stable tracking

Week 3 — Connectivity
WiFi connection
Server communication

Week 4 — Navigation
Pathfinding logic integration
System-level testing

Hardware Components (Parts List)
The following components are used in this project:

Microcontroller & Power
2 × ESP32 development boards
2 × USB isolators
2 × Batteries (1 charging)
1 × DC-to-DC converter
1 × Battery connection lead and fuse
2 × Spare fuses
1 × On/off switch

Motor & Drive System
2 × DC motors
2 × Wheels
1 × Rollerball caster
1 × DRV8835 dual motor driver
1 × DF Robot chassis with brackets

Sensors
1 × Ultrasonic sensor
1 × Sharp distance sensor + cable
1 × Line follower board

Prototyping & Wiring
1 × Breadboard
3 × 2-pin screw terminal connectors
2 × Packs of connection leads
1 × USB extension lead

Passive Components & Protection
1 × Protection diode
1 × Resistor network (1 kΩ)
1 × 10 kΩ resistor
1 × 150 Ω resistor
1 × Piezo buzzer

Mechanical Hardware
Nylon spacers (6 mm and 12 mm)
Assorted nylon bolts, washers, and nuts
Angular mounting pieces
Black card sheet
