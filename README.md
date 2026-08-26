# Design and Closed-Loop Control of a DC-DC Buck Converter

**Course Project (EE360) | MATLAB/Simulink-ready**

## Objective
Design and analyze a 24 V to 12 V, 60 W buck converter operating at 50 kHz, then regulate the output using a closed-loop PI controller.

## Specifications
- Input voltage: 24 V nominal
- Output voltage: 12 V
- Rated output power: 60 W
- Switching frequency: 50 kHz
- Load resistance: 2.4 ohm
- Inductor: 100 uH
- Output capacitor: 470 uF
- Nominal duty ratio: 0.5

## What is implemented
1. Component sizing from buck-converter equations.
2. Averaged state-space model of the converter.
3. Closed-loop digital PI voltage control with duty-cycle saturation.
4. Input-voltage and load-step disturbances.
5. Inductor-current, output-voltage and duty-cycle plots.
6. Transient metrics and theoretical ripple estimates.
7. Estimated efficiency using conduction/switching-loss assumptions.

## MATLAB requirements
MATLAB R2020b+ is recommended. The main script uses base MATLAB functionality; Control System Toolbox is not required.

## Run
Open MATLAB in this folder and execute:

```matlab
buck_converter_project
```

The script creates a `figures/` directory and saves the generated plots as PNG files.
