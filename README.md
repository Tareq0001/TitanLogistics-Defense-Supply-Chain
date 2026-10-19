# TitanLogistics — Strategic Defense Supply Chain & Fleet Readiness Twin

[![Live Demo](https://img.shields.io/badge/Live_Logistics_Twin-Tareq0001.github.io-blue?style=for-the-badge&logo=github)](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)
[![Readiness Standard](https://img.shields.io/badge/Military_Readiness-Ao_Operational_Availability-emerald?style=for-the-badge)](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)
[![AI Reliability](https://img.shields.io/badge/Predictive_Maintenance-Weibull_Distribution_%7C_RUL-orange?style=for-the-badge)](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)

> **A mission-critical strategic defense logistics, multi-echelon supply chain network, and combat fleet readiness digital twin. Features interactive graph-based resupply convoy routing (Dijkstra algorithm with threat aversion weights), real-time Operational Availability ($A_o$) tracking across Armored Brigades, Fighter Wings, and Naval Frigates, Weibull failure distribution modeling for predictive maintenance, and critical JP-8 fuel & precision munitions burn-rate forecasting.**

---

## ⚡ Core Defense Logistics Capabilities

1. **3D Multi-Echelon Strategic Logistics Network (Three.js)**
   - Interactive theater map modeling Central Depots, Forward Operating Bases (FOBs), Airbases, and Naval Ports.
   - Dynamic 3D military convoy movement along resupply corridors with safety escort status.

2. **Fleet Combat Operational Availability ($A_o$)**
   - Real-time mission-capable calculation across military platforms:
     $$A_o = \frac{\text{MTBF}}{\text{MTBF} + \text{MTTR}}$$
   - Multi-category fleet selection: Armored Brigades (M1A2S Abrams, Bradley IFV), Fighter Wings (F-15SA, Typhoon), and Multi-Role Guided Missile Frigates.

3. **Weibull Distribution Predictive Maintenance Engine**
   - Statistical failure modeling for turbine engines, mechanical gearboxes, and radar electronics:
     $$f(t) = \frac{\beta}{\eta} \left(\frac{t}{\eta}\right)^{\beta-1} e^{-(t/\eta)^\beta}$$
   - Remaining Useful Life (RUL) estimation triggering pre-emptive depot-level overhaul before catastrophic in-field failure occurs.

4. **Graph-Based Resupply Convoy Router (Dijkstra Optimization)**
   - Calculates the optimal and safest path between central distribution centers and forward frontline units avoiding air defense contested corridors.

---

## 🛠️ Architecture & Tech Stack

- **Graphics:** Three.js (r128), Canvas 2D Weibull probability curves.
- **Algorithms:** Dijkstra shortest path with dynamic risk penalties, Weibull 2-parameter reliability calculus.
- **Audio:** Web Audio API diesel turbine hum and dispatch chime synthesis.
- **UI:** Tactile glassmorphism defense logistics console, bilingual English/Arabic interface.

---

## 🚀 Live Deployment

- **Online Defense Logistics Twin:** [https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)
- **Author:** Eng. Tareq Aboushi (Tareq0001)
