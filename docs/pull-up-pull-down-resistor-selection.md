# Pull-Up and Pull-Down Resistor Selection Guide

## Purpose

A pull-up or pull-down resistor gives a digital signal a known default logic level when no device is actively driving it.

Unlike I2C or other communication buses, there is usually no timing specification that determines the resistor value. Instead, the resistor is chosen to balance:

1. Reliable default logic state
2. Low power consumption
3. Adequate noise immunity
4. Ability of the driving device to easily override the resistor

## Step 1 - Determine the Required Default State

First decide what the signal should do when nothing is driving it.

| Signal Type | Default State |
|---|---|
| Chip Select, active low | HIGH |
| Reset, active low | HIGH |
| Enable, active high | LOW |
| Enable, active low | HIGH |
| Interrupt, open drain | HIGH |
| Pushbutton input | HIGH or LOW depending on switch wiring |
| MOSFET gate | OFF |
| Boot configuration pin | Defined by datasheet |

## Step 2 - Calculate Static Current

Whenever the signal is actively driven opposite the resistor, current flows through the resistor.

Use Ohm's law:

```text
I = V / R
```

Example for a 3.3 V system:

| Resistor | Current when overridden |
|---:|---:|
| 1 kOhm | 3.3 mA |
| 2.2 kOhm | 1.5 mA |
| 4.7 kOhm | 0.70 mA |
| 10 kOhm | 0.33 mA |
| 22 kOhm | 0.15 mA |
| 47 kOhm | 70 uA |
| 100 kOhm | 33 uA |
| 1 MOhm | 3.3 uA |

Lower resistance gives a stronger default state but wastes more current.

Higher resistance saves power but is more susceptible to leakage and noise.

## Step 3 - Consider Signal Speed

The resistor and input capacitance form an RC network.

For slowly changing signals this usually does not matter.

For fast edges or open-drain outputs it can.

Typical guidance:

| Signal class | Typical range |
|---|---:|
| Very fast digital signals | 1-10 kOhm |
| Normal control signals | 10-47 kOhm |
| Battery-powered control signals | 47-100 kOhm |
| Ultra-low-power inputs | 100 kOhm-1 MOhm, verify leakage and noise margins |

## Typical Control Signals

### Chip Select / CS

Purpose:

Keep peripherals deselected during MCU reset.

Typical value:

- 10 kOhm

Battery-powered option:

- 47-100 kOhm if startup timing allows

### Reset / RST / RESET

Purpose:

Prevent accidental resets from noise.

Typical value:

- 10 kOhm

This is common in many MCU reference designs.

### Enable / EN

Purpose:

Force a regulator or peripheral into a known state at power-up.

Typical value:

- 10-100 kOhm

Use lower values if noise immunity is important.

### Interrupt / INT / IRQ

If the interrupt output is push-pull:

- Usually no external resistor is required.

If the interrupt output is open-drain or open-collector:

- A pull-up is required.

Typical values:

| Interrupt type | Typical pull-up |
|---|---:|
| Fast interrupt | 4.7-10 kOhm |
| Slow interrupt | 10-47 kOhm |

### Pushbutton Inputs

External pull-up:

- 10-47 kOhm

Battery-powered products:

- 47-100 kOhm

Internal MCU pull-ups:

- Often acceptable if precise resistance is not required.

### MOSFET Gates

Purpose:

Prevent unintended turn-on while the MCU is resetting or unpowered.

Typical values:

- 10-100 kOhm

### Boot Configuration Pins

Always follow the MCU datasheet.

These resistors often determine the boot mode and should not be changed without understanding the consequences.

### Latch / HOLD / Power-Hold Signals

Examples:

- Power latch
- Power hold
- Shutdown request
- Enable feedback

These values depend on the latch circuit.

Typical range:

- 47-100 kOhm

## Choosing a Value

Start with:

- 10 kOhm

Then ask:

- Does this waste too much current?
- Is the line exposed to electrical noise?
- Is the signal open-drain?
- Does the line need fast edges?
- Is this a battery-powered product?

Adjust accordingly.

## Quick Reference

| Application | Typical Value |
|---|---:|
| Reset | 10 kOhm |
| Chip Select | 10 kOhm |
| Open-drain Interrupt | 10 kOhm |
| Pushbutton | 10-47 kOhm |
| Enable | 10-100 kOhm |
| MOSFET Gate | 10-100 kOhm |
| Power Hold | 47-100 kOhm |
| Boot Pins | Datasheet value |
| Ultra-low-power inputs | 100 kOhm-1 MOhm, verify carefully |

## General Rules

- Never rely on an input floating.
- Use the weakest resistor that still provides reliable operation.
- Lower resistance improves noise immunity but increases current consumption.
- Higher resistance reduces current but increases sensitivity to leakage and EMI.
- Always check the datasheet for any signal with special startup or boot requirements.
