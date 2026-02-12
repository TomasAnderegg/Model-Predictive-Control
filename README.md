# MPC Cruise Controller for a Car on a Highway

**ME-425 Mini-Project** | EPFL  
Tomas Garate Anderegg · Matas Antanas Jones · Louis Gilles Canoen

This project implements Model Predictive Control (MPC) strategies for a simulated VW ID.3 driving on a highway. The controller handles cruising, lane changes, and overtaking while ensuring passenger comfort and safety.

<p align="center">
  <img src="images/car_model.png" alt="Car Model" width="50%">
</p>

---

## Deliverables

### Deliverable 2 — Linearization

Linearization of the four-state kinematic car model around steady-state conditions. The system is decoupled into **longitudinal** (speed control via throttle) and **lateral** (lane control via steering) subsystems, enabling independent controller design.

<p align="center">
  <img src="media/deliverable_2.gif" alt="Linearization Demo" width="60%">
</p>

---

### Deliverable 3 — Linear MPC Design

Design of the linear MPC controller with tuned Q, R matrices and horizon length. The controller satisfies:
- Acceleration from 80 → 120 km/h in under 10 s
- Lane change in under 3 s

A terminal invariant set is computed to guarantee recursive feasibility.

<p align="center">
  <img src="media/deliverable_3.gif" alt="Linear MPC Demo" width="60%">
</p>

---

### Deliverable 4 — Offset-Free Tracking

Integration of a state and disturbance estimator to eliminate steady-state tracking error introduced by linearization. Observer gains are tuned via pole placement to balance convergence speed and stability.

<p align="center">
  <img src="media/deliverable_4.gif" alt="Offset-Free Tracking Demo" width="60%">
</p>

---

### Deliverable 5 — Robust Tube MPC

Robust tube MPC for safe car-following with a lead vehicle. A minimal robust invariant set is computed to handle disturbances from the lead car's throttle variations. Tightened constraints ensure collision avoidance with a safe distance x_safe = 10 m.

**Case 1** — Ego car catches up to a slower lead car  
**Case 2** — Lead car brakes suddenly and re-accelerates

<p align="center">
  <img src="media/deliverable_5.gif" alt="Robust Tube MPC Demo" width="60%">
</p>

---

### Deliverable 6.1 — Nonlinear MPC

Nonlinear MPC using RK4 integration instead of linearization, enabling higher precision across the full operating range. No steady-state tracking error.

<p align="center">
  <img src="media/deliverable_6_1.gif" alt="NMPC Demo" width="60%">
</p>

---

### Deliverable 6.2 — Nonlinear MPC with Overtaking

Extension of the NMPC with an ellipsoidal anti-collision constraint for safe overtaking maneuvers (semi-major axis a = 15 m, semi-minor axis b = 3 m).

<p align="center">
  <img src="media/deliverable_6_2.gif" alt="NMPC Overtaking Demo" width="60%">
</p>

---

## Project Structure
```
├── src/                  # MATLAB source files
├── media/                # Demo videos/GIFs
├── images/               # Figures and diagrams
├── report/               # Project report (PDF)
└── README.md
```

## How to Run

1. Open MATLAB and navigate to the project directory
2. Run the desired deliverable script (e.g., `deliverable_3.m`)
3. The simulation will display the car's behavior and plot the results

## References

- ME-425 Model Predictive Control, EPFL
- VW ID.3 kinematic car model provided as part of the course
