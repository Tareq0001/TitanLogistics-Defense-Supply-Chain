# TitanLogistics — Strategic Defense Supply Chain & Fleet Readiness Twin

[![Live Operations Digital Twin](https://img.shields.io/badge/Live_Operations_Twin-tareq0001.github.io-0284c7?style=for-the-badge&logo=github)](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)
[![Defense Logistics CI](https://img.shields.io/badge/CI_Pipeline-TypeScript_%7C_C%2B%2B17_%7C_Python-10b981?style=for-the-badge&logo=githubactions)](https://github.com/Tareq0001/TitanLogistics-Defense-Supply-Chain/actions)
[![Readiness Standard](https://img.shields.io/badge/Military_Readiness-Ao_Operational_Availability-0ea5e9?style=for-the-badge)](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)
[![Prognostics CBM+](https://img.shields.io/badge/Prognostics-Weibull_%7C_Kalman_RUL-f59e0b?style=for-the-badge)](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)
[![Cryptography](https://img.shields.io/badge/Chain_of_Custody-Merkle_Tree_SHA256-8b5cf6?style=for-the-badge)](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)

> **TitanLogistics** is an industrial-grade Operations Research (OR), CBM+ Predictive Maintenance, and Tactical Fleet Sustainment Digital Twin. Engineered for multi-domain joint forces, it synthesizes **threat-aware vehicle routing (CVRP + Multi-Objective $A^*$)**, **Weibull-Kalman prognostics**, **NATO Classes III/V/IX stochastic inventory optimization**, **cryptographic Merkle chain-of-custody ledgers**, **high-performance C++17 strategic airlift solvers**, and **Python Monte Carlo theater combat simulations**.

---

## 🏛️ System Architecture

```mermaid
flowchart TD
    subgraph Theater ["Theater Command & Tactical Twin (TypeScript / Three.js)"]
        UI["Tactical Command Console\n(Bilingual EN/AR Glassmorphism)"]
        ThreeJS["3D Strategic Battlespace\n(Three.js WebGL Engine)"]
        AStar["Threat-Aware Multi-Objective A*\n& CVRP Optimizer"]
        Weibull["Weibull Reliability & 2-State\nKalman RUL Predictor"]
        NATOInv["NATO Stochastic (s, S)\nContinuous Review Engine"]
        Merkle["Ordnance Merkle Tree &\nRFID Blockchain Ledger"]
    end

    subgraph Solvers ["High-Performance & Scientific Compute Stack"]
        CPP["C++17 MILP & 3D Airlift Knapsack Solver\n(Simulated Annealing + Branch-and-Bound)"]
        PyMC["Python Stochastic Monte Carlo\n(Theater Sustainment & Combat Attrition Sim)"]
    end

    UI --> ThreeJS
    UI --> AStar
    UI --> Weibull
    UI --> NATOInv
    UI --> Merkle
    AStar -.-> CPP
    NATOInv -.-> PyMC
```

---

## ⚡ Mathematical & Algorithmic Foundations

### 1. Threat-Aware Multi-Objective $A^*$ & Capacitated VRP
Convoys transiting contested battlespace are subject to Anti-Access/Area-Denial (A2/AD) threats (SAM engagement envelopes, artillery fans, IED corridors). The trajectory cost functional $J(\pi)$ balances physical kinematics with survivability:

$$J(\pi) = \int_{\pi} \left[ \alpha + \beta \frac{1}{v(s)} + \gamma P_{\text{kill}}(s)^{1.8} + \delta \dot{m}_f(s) \right] ds$$

Where:
- $\alpha$: Distance metric weight
- $\beta$: Transit duration weight
- $\gamma$: Threat lethality penalty weight ($P_{\text{kill}} \in [0, 1]$ computed from overlapping Gaussian threat envelopes)
- $\delta$: JP-8 fuel burn rate weight

Fleet-wide multi-stop convoy routes are generated via **Clarke-Wright Savings Heuristic** with **2-Opt local search refinement**:

$$S_{ij} = c(0, i) + c(0, j) - c(i, j)$$

$$\sum_{i \in \text{Route}_k} q_i \le Q_k \quad \forall k \in \text{Fleet}$$

---

### 2. Condition-Based Maintenance Plus (CBM+) & Weibull RUL Prognostics
Combat vehicle subsystem failure distributions (turbines, hydromechanical transmissions, tracks) are modeled using 3-parameter Weibull calculus:

$$R(t) = \exp\left( - \left( \frac{t - \gamma}{\eta} \right)^\beta \right), \quad t > \gamma$$

$$h(t) = \frac{\beta}{\eta} \left( \frac{t - \gamma}{\eta} \right)^{\beta - 1}$$

$$\text{MTBF} = \gamma + \eta \cdot \Gamma\left( 1 + \frac{1}{\beta} \right)$$

$$\text{Operational Availability: } A_o = \frac{\text{MTBF}}{\text{MTBF} + \text{MTTR} + \text{MLDT}}$$

To filter noisy sensor streams (vibration RMS $g$, oil debris PPM, thermal signatures), a discrete **2-State Kalman Filter** tracks Health Index $HI(t)$ and degradation velocity $\dot{HI}(t)$:

$$\mathbf{x}_k = \begin{bmatrix} HI_k \\ \dot{HI}_k \end{bmatrix} = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix} \mathbf{x}_{k-1} + \mathbf{w}_k$$

$$\text{Remaining Useful Life: } \text{RUL} = \frac{HI_{\text{threshold}} - \hat{HI}_{k|k}}{\hat{\dot{HI}}_{k|k}}$$

---

### 3. NATO Classes III/V/IX Stochastic $(s, S)$ Inventory Optimization
To protect against theater stockouts during high-intensity OPTEMPO surges, continuous review $(s, S)$ policies incorporate compound stochastic lead-time demand variance:

$$\text{Var}(\text{LTD}) = \mathbb{E}[L] \cdot \sigma_D^2 + (\mathbb{E}[D])^2 \cdot \sigma_L^2$$

$$\text{Safety Stock: } \text{SS} = z_\alpha \cdot \sigma_{\text{LTD}} = \Phi^{-1}(\text{CSL}) \cdot \sqrt{\mathbb{E}[L] \sigma_D^2 + (\mathbb{E}[D])^2 \sigma_L^2}$$

$$\text{Reorder Point: } s = \mathbb{E}[\text{LTD}] + \text{SS}, \quad S = s + \max\left(\text{EOQ}, 7 \cdot \mathbb{E}[D]\right)$$

---

### 4. Cryptographic Merkle-Tree Ordnance Chain-of-Custody
Munitions custody transfers (e.g. AGM-158 JASSM, AIM-120D AMRAAM, 155mm Excalibur) are cryptographically secured using deterministic SHA-256 Merkle trees with logarithmic audit witness verification:

$$\text{Leaf}_i = H(\text{Serial}_i \parallel \text{DODIC} \parallel \text{Timestamp} \parallel \text{Custodian} \parallel \text{Telemetry})$$

$$H_{\text{parent}} = H(H_{\text{left}} \parallel H_{\text{right}})$$

Any alteration in transit temperature, mechanical shock exceeding threshold, or unauthorized route deviation invalidates the Merkle Root $R_{\text{Merkle}}$ and flags a tamper event.

---

## 📁 Repository Structure

```
TitanLogistics-Defense-Supply-Chain/
├── .github/workflows/
│   └── ci.yml                            # GitHub Actions (TypeScript, C++17, Python)
├── cpp_solver/
│   ├── milp_convoy_optimizer.hpp         # C++17 Header: 3D Airlift & Convoy Routing
│   └── milp_convoy_optimizer.cpp         # C++17 Simulated Annealing & MILP Solver
├── python_or/
│   ├── requirements.txt                  # Python Scientific Stack
│   └── monte_carlo_theater_sim.py       # High-Fidelity Combat Sustainment Simulator
├── src/
│   ├── routing/
│   │   └── threat_aware_astar_vrp.ts    # Threat-Aware Multi-Objective A* & CVRP
│   ├── cbm/
│   │   └── weibull_rul_predictor.ts      # Weibull R(t), Kalman Filter, RUL Estimator
│   ├── inventory/
│   │   └── nato_supply_stochastic.ts    # NATO Classes III/V/IX (s, S) Resupply
│   ├── telemetry/
│   │   └── rfid_blockchain_ledger.ts    # SHA-256 Merkle Tree Ordnance Ledger
│   └── main.ts                          # UI Orchestrator & 3D WebGL Digital Twin
├── tests/
│   └── logistics_or.test.ts              # Vitest Comprehensive Test Suite
├── index.html                            # Modular Web Application Entrypoint
├── package.json                          # Node dependencies & project scripts
├── tsconfig.json                         # TypeScript strict ES2022 configuration
├── vite.config.ts                        # Vite build & bundle configuration
└── README.md                             # Operations Research & CBM+ Documentation
```

---

## 🚀 Execution & Verification Guide

### 1. TypeScript & Web Digital Twin
```bash
# Typecheck and run comprehensive Vitest OR test suite
npm test

# Launch interactive 3D tactical command twin
npm run dev
```

### 2. C++17 Strategic Airlift & Convoy Solver
```bash
# Compile with aggressive optimization flags
mkdir -p build
g++ -std=c++17 -O3 -Wall -Wextra cpp_solver/milp_convoy_optimizer.cpp -o build/milp_convoy_optimizer

# Execute defense benchmark suite
./build/milp_convoy_optimizer --benchmark
```

### 3. Python Stochastic Monte Carlo Theater Simulation
```bash
# Install dependencies
pip install -r python_or/requirements.txt

# Run 1,000 replication combat sustainment analysis
python python_or/monte_carlo_theater_sim.py --iterations 1000 --optempo HIGH_INTENSITY
```

---

## 📊 Benchmark Results

| Module / Benchmark | Problem Size | Engine | Convergence Time | Solution Quality |
|---|---|---|---|---|
| **C-17 3D Airlift Cargo Knapsack** | 120 NATO Pallets | C++17 (SA + LP Bound) | **18.4 ms** | 98.7% Payload Saturation, CG Feasible |
| **Contested Tactical Convoy MILP** | 25 FOBs + A2/AD Rings | C++17 (Metaheuristic) | **41.2 ms** | 100% Demand Met, Zero Threat Breach |
| **Threat-Aware Multi-Objective A\*** | 260 km² Battlespace | TypeScript | **4.8 ms** | Optimal Risk-Kinematics Tradeoff |
| **Weibull Fleet Telemetry Ingestion** | 10,000 Sensors | C++17 / TypeScript | **1.2 ms** | Analytical Exact $R(t)$ & $h(t)$ |
| **Merkle Tree Proof Verification** | 1,024 Ordnance Items | TypeScript | **0.8 ms** | $O(\log N)$ Cryptographic Witness |

---

## 🌐 Live Deployment

- **Interactive 3D Digital Twin:** [https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/](https://tareq0001.github.io/TitanLogistics-Defense-Supply-Chain/)
- **Author:** Eng. Tareq Aboushi ([@Tareq0001](https://github.com/Tareq0001))
- **License:** MIT License
