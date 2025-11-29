# Cruise Control System: PID vs LQR (EE 586 Project)

This repository contains a MATLAB/Simulink implementation of a cruise control system using:
- A GA‑optimized PID controller
- An LQR‑ITAE optimal controller

The project replicates and compares results from the reference paper:

> *Comparison between Genetic Algorithms of Proportional–Integral–Derivative and Linear Quadratic Regulator Controllers, and Fuzzy Logic Controllers for Cruise Control System*, WEVJ.

---

## Folder Structure

```
cruise-control-lqr-pid/
├─ README.md
├─ docs/
│  ├─ EE_586_Project_Proposal.pdf
│  └─ wevj-15-00351.pdf
├─ src/
│  ├─ step_response/
│  │  ├─ pid_step.m
│  │  ├─ lqr_step.m
│  │  ├─ step_compare_all.m
│  │  ├─ step_metrics.m
│  │  ├─ pid_step_design.slx
│  │  └─ lqr_step_design.slx
│  │
│  ├─ acc_dec/
│     ├─ pid_accdec.m
│     ├─ lqr_accdec.m
│     ├─ pid_accdec_design.slx
│     ├─ lqr_accdec_design.slx
│     └─ accdec_compare_all.m
│
└─ figures/
   ├─ step/
   │  ├─ pid_step.png
   │  ├─ lqr_step.png
   │  └─ step_compare_all.png
   └─ acc_dec/
      ├─ pid_accdec.png
      ├─ lqr_accdec.png
      └─ accdec_compare_all.png
```

---

## How to Run

Open MATLAB and add the `src` folder to your path:

```matlab
addpath(genpath('src'));
```

### Step Responses (25 seconds)
- PID step:  
  ```matlab
  run('src/step_response/pid_step.m');
  ```
- LQR step:  
  ```matlab
  run('src/step_response/lqr_step.m');
  ```
- Compare PID vs LQR vs Original:
  ```matlab
  run('src/step_response/step_compare_all.m');
  ```

---

### Acceleration/Deceleration Simulation (0–60 seconds)
- PID acc/dec:
  ```matlab
  run('src/acc_dec/pid_accdec.m');
  ```
- LQR acc/dec:
  ```matlab
  run('src/acc_dec/lqr_accdec.m');
  ```
- Compare PID vs LQR vs Reference:
  ```matlab
  run('src/acc_dec/accdec_compare_all.m');
  ```

---

## Metrics Computation (ITAE, ITSE, etc.)
The file:

```
src/step_response/step_metrics.m
```

will contain all performance metric calculations for:
- Rise time  
- Settling time  
- Overshoot  
- Undershoot  
- ITAE  
- ITSE  
- IAE  
- ISE  

---

## Requirements

- MATLAB R2022a or newer  
- Control System Toolbox  
- Simulink (for .slx models)

---

## Documentation

See the **docs/** folder for:
- Project report  
- Reference paper (WEVJ)

---

