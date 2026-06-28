# Decoupling and Bypass Capacitor Selection Guide

## Purpose

Decoupling (bypass) capacitors provide a local energy reservoir for an IC during rapid current changes.

They supply short bursts of current, prevent voltage dips, reduce high-frequency noise, and isolate switching noise between ICs.

## Two Types of Capacitors

### Local Bypass Capacitor

Located immediately beside each power pin.

Purpose:
- Supply very fast transient current
- Filter high-frequency switching noise

Typical value:
- 100 nF ceramic (X7R)

### Bulk Decoupling Capacitor

Located near regulators or groups of ICs.

Purpose:
- Supply lower-frequency current
- Handle larger load changes
- Support the local power rail

Typical values:
- 1 uF
- 4.7 uF
- 10 uF
- 22 uF

## Choosing Capacitor Values

| Frequency range | Typical capacitor |
|---|---|
| Hundreds of MHz | 10-100 nF |
| Tens of MHz | 100 nF |
| Hundreds of kHz to MHz | 1-10 uF |
| Low-frequency load changes | 10-100 uF |

## Typical Components

### Microcontrollers
- 100 nF per VDD pin
- 4.7-10 uF nearby

### Sensors
- 100 nF
- Some analog sensors also require 1 uF

### Analog ICs
- 100 nF
- 1 uF nearby

### Switching Regulators
Follow the datasheet.
Typical:
- Input: 10-22 uF
- Output: 10-22 uF

### LDO Regulators
Follow the datasheet.
Typical output capacitor:
- 1-10 uF

### Battery Connections
- 4.7-22 uF

### Power Connectors
- 10-47 uF

## Placement Rules

Local bypass capacitors:
- Immediately beside the power pin
- Same PCB side when possible
- Short trace to power
- Short trace to ground
- Direct connection into the ground plane

Ideal distance:
- Less than 2-3 mm

Bulk capacitors:
- Near regulator
- Near power entry
- Near groups of ICs

## Dielectric

Preferred:
- X7R

Acceptable:
- X5R

Avoid:
- Y5V
- Z5U

## Voltage Rating

Use at least twice the operating voltage when practical.

Typical:
- 3.3 V rail -> 6.3 V, 10 V or 16 V capacitor.

## Package Size

General recommendation:
- 0603

## Quick Reference

| Application | Typical capacitor |
|---|---|
| MCU VDD | 100 nF |
| Sensor supply | 100 nF |
| Analog IC | 100 nF + 1 uF |
| MCU bulk | 4.7-10 uF |
| Regulator input | Datasheet (typically 10-22 uF) |
| Regulator output | Datasheet (typically 10-22 uF) |
| Battery input | 10-22 uF |
| USB VBUS | 10 uF |
| Power connector | 10-47 uF |

## General Rules

- Every power pin gets its own bypass capacitor.
- Placement is usually more important than capacitance.
- Use X7R ceramics unless the datasheet specifies otherwise.
- Always follow regulator and analog IC datasheets before applying general rules.