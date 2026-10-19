#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <random>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <memory>

namespace TitanLogistics {

// ============================================================================
// 1. DATA STRUCTURES & DEFINITIONS
// ============================================================================

enum class CargoClass {
    CLASS_I_SUBSISTENCE,
    CLASS_III_POL_FUEL,
    CLASS_V_AMMUNITION,
    CLASS_IX_REPAIR_PARTS
};

struct AirliftItem {
    int id;
    std::string nomenclature;
    CargoClass cargoClass;
    double weightKg;
    double volumeM3;
    double footprintM2;
    double tacticalPriorityScore; // e.g., 100 for high-priority PGMs, 20 for rations
    bool isHazmatMunitions;
};

struct AircraftHoldCapacity {
    std::string aircraftModel;   // "C-130J_SUPER_HERCULES" or "C-17_GLOBEMASTER_III"
    double maxPayloadKg;
    double maxVolumeM3;
    double maxFloorAreaM2;
    double cgAllowableMinM;     // Center of Gravity allowable envelope [m]
    double cgAllowableMaxM;
};

struct KnapsackSolution {
    std::vector<int> selectedItemIds;
    double totalWeightKg;
    double totalVolumeM3;
    double totalFootprintM2;
    double totalPriorityScore;
    double centerOfGravityM;
    bool isCgFeasible;
    double solveDurationMs;
};

struct TacticalBaseNode {
    int id;
    std::string callsign;
    double posX;
    double posY;
    double demandTons;
    double serviceTimeHours;
    double threatRiskFactor; // A2/AD vulnerability [0.0 = safe, 1.0 = lethal]
};

struct ConvoyOptimizationResult {
    std::vector<std::vector<int>> vehicleRoutes;
    double totalDistanceKm;
    double totalFuelBurnLiters;
    double totalThreatExposureCost;
    double objectiveCost;
    bool allDemandsSatisfied;
    double solveDurationMs;
};

// ============================================================================
// 2. 3D AIRLIFT MULTI-DIMENSIONAL KNAPSACK SOLVER
// ============================================================================

class AirliftKnapsackOptimizer {
private:
    AircraftHoldCapacity hold;
    std::vector<AirliftItem> inventory;

public:
    AirliftKnapsackOptimizer(const AircraftHoldCapacity& capacity, const std::vector<AirliftItem>& items);

    // Hybrid Simulated Annealing + Greedy LP-Relaxation Bound
    KnapsackSolution solve(int maxIterations = 20000, double initialTemp = 1000.0, double coolingRate = 0.998);
};

// ============================================================================
// 3. TACTICAL CONVOY MILP / HEURISTIC ROUTE OPTIMIZER
// ============================================================================

class ConvoyMilpOptimizer {
private:
    std::vector<TacticalBaseNode> bases;
    double vehicleCapacityTons;
    int fleetSize;
    double threatPenaltyWeight;

    double computeDistance(const TacticalBaseNode& a, const TacticalBaseNode& b) const;
    double evaluateRouteCost(const std::vector<int>& route) const;

public:
    ConvoyMilpOptimizer(const std::vector<TacticalBaseNode>& baseNodes, 
                        double maxTruckPayloadTons = 22.0, 
                        int availableFleet = 8,
                        double threatWeight = 45.0);

    ConvoyOptimizationResult solve(int maxEpochs = 35000);
};

// ============================================================================
// 4. BENCHMARK SUITE RUNNER
// ============================================================================

class DefenseLogisticsBenchmark {
public:
    static void runAirliftKnapsackBenchmark();
    static void runConvoyRoutingBenchmark();
    static void runWeibullFleetBenchmark();
    static int executeAll();
};

} // namespace TitanLogistics
