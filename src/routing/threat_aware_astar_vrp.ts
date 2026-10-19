/**
 * TitanLogistics Defense Supply Chain Engine
 * Threat-Aware Multi-Objective A* Convoy Pathfinder & Capacitated Vehicle Routing Problem (CVRP)
 * 
 * Implements tactical route generation through contested battlespace environments,
 * evaluating kinematic distance, fuel consumption, terrain slope, and anti-access/area-denial (A2/AD)
 * threat lethality envelopes (SAM rings, IED corridors, artillery counter-battery zones).
 */

export interface Point3D {
  x: number;
  y: number;
  z: number;
}

export interface TheaterNode extends Point3D {
  id: string;
  name: string;
  type: 'MAIN_LOGISTICS_BASE' | 'FORWARD_OPERATING_BASE' | 'COMBAT_OUTPOST' | 'TACTICAL_WAYPOINT';
  demands: {
    classIII_fuel_gal: number;       // JP-8 / F-34 military kerosene
    classV_ammo_tons: number;        // Guided PGMs, artillery rounds, small arms
    classIX_spares_kg: number;       // Critical avionics, powertrains, turbine blades
  };
  timeWindow?: {
    earliest: number; // minutes from mission epoch H-0
    latest: number;
  };
  serviceDurationMin: number;
}

export interface ThreatZone {
  id: string;
  name: string;
  center: Point3D;
  radiusKm: number;
  lethalityProbability: number; // P_k in [0.0, 1.0]
  type: 'A2AD_SAM_RING' | 'ARTILLERY_FIRE_FAN' | 'CONTESTED_IED_CORRIDOR' | 'DRONE_SWARM_ZONE';
}

export interface PathfindingWeights {
  distanceAlpha: number;      // weight on physical road distance
  transitTimeBeta: number;    // weight on transit duration
  threatExposureGamma: number;// weight on threat lethality penalty
  fuelBurnDelta: number;      // weight on logistical fuel consumption
}

export interface ConvoyVehicle {
  id: string;
  callsign: string;
  model: 'HEMTT_M978_TANKER' | 'OSHKOSH_PLS_M1075' | 'FMTV_M1083_CARGO' | 'C130J_AIRLIFT';
  maxPayloadTons: number;
  maxFuelCapacityGal: number;
  operationalRangeKm: number;
  avgSpeedKph: number;
  fuelBurnRateGalPerKm: number;
  armorProtectionLevel: number; // 1-5 STANAG 4569 rating
}

export interface ConvoyLeg {
  fromNodeId: string;
  toNodeId: string;
  distanceKm: number;
  transitTimeMin: number;
  fuelBurnedGal: number;
  accumulatedRisk: number;
  waypoints: Point3D[];
}

export interface ConvoyRoute {
  vehicleId: string;
  vehicleModel: string;
  assignedNodes: TheaterNode[];
  legs: ConvoyLeg[];
  totalDistanceKm: number;
  totalTransitTimeMin: number;
  totalFuelBurnedGal: number;
  maxLegRisk: number;
  deliveredSupplies: {
    classIII_fuel_gal: number;
    classV_ammo_tons: number;
    classIX_spares_kg: number;
  };
  feasible: boolean;
}

export interface CVRPSolution {
  depotId: string;
  routes: ConvoyRoute[];
  totalFleetDistanceKm: number;
  totalTheaterFuelBurnGal: number;
  totalTheaterRiskIndex: number;
  unassignedNodes: TheaterNode[];
  computationTimeMs: number;
}

export class ThreatAwareAStarPathfinder {
  private weights: PathfindingWeights;

  constructor(weights: Partial<PathfindingWeights> = {}) {
    this.weights = {
      distanceAlpha: weights.distanceAlpha ?? 1.0,
      transitTimeBeta: weights.transitTimeBeta ?? 0.8,
      threatExposureGamma: weights.threatExposureGamma ?? 4.5,
      fuelBurnDelta: weights.fuelBurnDelta ?? 0.6,
    };
  }

  /**
   * Euclidean metric in 3D theater space
   */
  public static calculateDistance(p1: Point3D, p2: Point3D): number {
    const dx = p1.x - p2.x;
    const dy = p1.y - p2.y;
    const dz = (p1.z - p2.z) * 0.1; // elevation weighting
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  /**
   * Evaluates instantaneous threat risk at a 3D coordinate based on overlapping threat zones
   */
  public evaluateThreatRisk(point: Point3D, threatZones: ThreatZone[]): number {
    let combinedSurvivalProb = 1.0;

    for (const threat of threatZones) {
      const dist = ThreatAwareAStarPathfinder.calculateDistance(point, threat.center);
      if (dist <= threat.radiusKm) {
        // Linear-Gaussian attenuation toward perimeter
        const normalizedDist = dist / threat.radiusKm;
        const localLethality = threat.lethalityProbability * Math.exp(-2.5 * normalizedDist * normalizedDist);
        combinedSurvivalProb *= (1.0 - Math.min(0.99, localLethality));
      }
    }

    return 1.0 - combinedSurvivalProb; // Combined Threat Lethality Index P_kill
  }

  /**
   * Solves Multi-Objective optimal path between start and target avoiding A2/AD zones
   */
  public findPath(
    start: Point3D,
    goal: Point3D,
    threatZones: ThreatZone[],
    gridResolutionKm = 5.0,
    boundsPaddingKm = 20.0
  ): { path: Point3D[]; totalCost: number; distanceKm: number; threatExposure: number } {
    const minX = Math.min(start.x, goal.x) - boundsPaddingKm;
    const maxX = Math.max(start.x, goal.x) + boundsPaddingKm;
    const minY = Math.min(start.y, goal.y) - boundsPaddingKm;
    const maxY = Math.max(start.y, goal.y) + boundsPaddingKm;

    const key = (p: Point3D) => `${Math.round(p.x * 10) / 10},${Math.round(p.y * 10) / 10}`;

    const openSet: Array<{ point: Point3D; fScore: number }> = [{ point: start, fScore: 0 }];
    const cameFrom = new Map<string, Point3D>();
    const gScore = new Map<string, number>();
    gScore.set(key(start), 0);

    const heuristic = (p: Point3D): number => {
      const dist = ThreatAwareAStarPathfinder.calculateDistance(p, goal);
      const risk = this.evaluateThreatRisk(p, threatZones);
      return this.weights.distanceAlpha * dist + this.weights.threatExposureGamma * (risk * 50);
    };

    let iterations = 0;
    const maxIterations = 2500;
    let bestCandidate = start;
    let closestToGoal = ThreatAwareAStarPathfinder.calculateDistance(start, goal);

    while (openSet.length > 0 && iterations++ < maxIterations) {
      // Priority extraction
      openSet.sort((a, b) => a.fScore - b.fScore);
      const current = openSet.shift()!.point;
      const currentKey = key(current);

      const distToGoal = ThreatAwareAStarPathfinder.calculateDistance(current, goal);
      if (distToGoal < closestToGoal) {
        closestToGoal = distToGoal;
        bestCandidate = current;
      }

      if (distToGoal <= gridResolutionKm * 0.95) {
        // Goal reached within spatial threshold
        cameFrom.set(key(goal), current);
        bestCandidate = goal;
        break;
      }

      // Generate 8-directional neighbor movements
      const neighbors: Point3D[] = [
        { x: current.x + gridResolutionKm, y: current.y, z: current.z },
        { x: current.x - gridResolutionKm, y: current.y, z: current.z },
        { x: current.x, y: current.y + gridResolutionKm, z: current.z },
        { x: current.x, y: current.y - gridResolutionKm, z: current.z },
        { x: current.x + gridResolutionKm * 0.707, y: current.y + gridResolutionKm * 0.707, z: current.z },
        { x: current.x - gridResolutionKm * 0.707, y: current.y + gridResolutionKm * 0.707, z: current.z },
        { x: current.x + gridResolutionKm * 0.707, y: current.y - gridResolutionKm * 0.707, z: current.z },
        { x: current.x - gridResolutionKm * 0.707, y: current.y - gridResolutionKm * 0.707, z: current.z },
      ];

      for (const n of neighbors) {
        if (n.x < minX || n.x > maxX || n.y < minY || n.y > maxY) continue;

        const stepDist = ThreatAwareAStarPathfinder.calculateDistance(current, n);
        const threatLethality = this.evaluateThreatRisk(n, threatZones);

        // Edge transition cost function J_edge
        const edgeCost =
          this.weights.distanceAlpha * stepDist +
          this.weights.threatExposureGamma * (Math.pow(threatLethality, 1.8) * 120) +
          this.weights.fuelBurnDelta * (stepDist * 0.45);

        const currentG = gScore.get(currentKey) ?? Infinity;
        const tentativeG = currentG + edgeCost;
        const nKey = key(n);

        if (tentativeG < (gScore.get(nKey) ?? Infinity)) {
          cameFrom.set(nKey, current);
          gScore.set(nKey, tentativeG);
          const f = tentativeG + heuristic(n);

          const existingIndex = openSet.findIndex(item => key(item.point) === nKey);
          if (existingIndex >= 0) {
            openSet[existingIndex].fScore = f;
          } else {
            openSet.push({ point: n, fScore: f });
          }
        }
      }
    }

    // Reconstruct path backward
    const path: Point3D[] = [];
    let curr: Point3D | undefined = bestCandidate;
    let backSteps = 0;
    while (curr && backSteps++ < maxIterations) {
      path.unshift(curr);
      curr = cameFrom.get(key(curr));
    }

    if (path.length > 0 && (path[path.length - 1].x !== goal.x || path[path.length - 1].y !== goal.y)) {
      path.push(goal);
    }

    let totalDist = 0;
    let accumulatedThreat = 0;
    for (let i = 0; i < path.length - 1; i++) {
      totalDist += ThreatAwareAStarPathfinder.calculateDistance(path[i], path[i + 1]);
      accumulatedThreat += this.evaluateThreatRisk(path[i], threatZones);
    }

    return {
      path,
      totalCost: gScore.get(key(bestCandidate)) ?? totalDist,
      distanceKm: totalDist,
      threatExposure: path.length > 0 ? accumulatedThreat / path.length : 0,
    };
  }
}

export class CapacitatedVRPOptimizer {
  private depot: TheaterNode;
  private nodes: TheaterNode[];
  private fleet: ConvoyVehicle[];
  private threatZones: ThreatZone[];
  private pathfinder: ThreatAwareAStarPathfinder;

  constructor(
    depot: TheaterNode,
    nodes: TheaterNode[],
    fleet: ConvoyVehicle[],
    threatZones: ThreatZone[] = []
  ) {
    this.depot = depot;
    this.nodes = nodes.filter(n => n.id !== depot.id);
    this.fleet = fleet;
    this.threatZones = threatZones;
    this.pathfinder = new ThreatAwareAStarPathfinder();
  }

  /**
   * Distance and cost matrix calculation between two theater points
   */
  private getDistance(p1: Point3D, p2: Point3D): number {
    return ThreatAwareAStarPathfinder.calculateDistance(p1, p2);
  }

  /**
   * Clarke-Wright Savings Algorithm adapted for Multi-Commodity Military Supply Lines
   * Savings S_ij = c(0, i) + c(0, j) - c(i, j)
   */
  public solveClarkeWright(): CVRPSolution {
    const startTime = performance.now();
    const customers = [...this.nodes];
    const n = customers.length;

    // Calculate individual customer supply volume metric (tonnage & fuel)
    const getPayloadTons = (node: TheaterNode): number => {
      const fuelTons = (node.demands.classIII_fuel_gal * 3.1) / 1000; // JP-8 density ~3.1 kg/gal
      return node.demands.classV_ammo_tons + (node.demands.classIX_spares_kg / 1000) + fuelTons;
    };

    // Calculate Clarke-Wright Savings Matrix
    interface SavingPair {
      i: TheaterNode;
      j: TheaterNode;
      savings: number;
    }
    const savingsList: SavingPair[] = [];

    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const d0i = this.getDistance(this.depot, customers[i]);
        const d0j = this.getDistance(this.depot, customers[j]);
        const dij = this.getDistance(customers[i], customers[j]);
        const savings = d0i + d0j - dij;
        savingsList.push({ i: customers[i], j: customers[j], savings });
      }
    }

    // Sort descending by savings
    savingsList.sort((a, b) => b.savings - a.savings);

    // Initial individual routes: Depot -> Node_i -> Depot
    let activeRoutes: TheaterNode[][] = customers.map(c => [c]);

    // Merge routes based on vehicle payload & range constraints
    const maxVehiclePayload = Math.max(...this.fleet.map(f => f.maxPayloadTons));

    for (const pair of savingsList) {
      const routeIdxI = activeRoutes.findIndex(r => r[r.length - 1].id === pair.i.id);
      const routeIdxJ = activeRoutes.findIndex(r => r[0].id === pair.j.id);

      if (routeIdxI !== -1 && routeIdxJ !== -1 && routeIdxI !== routeIdxJ) {
        const combinedNodes = [...activeRoutes[routeIdxI], ...activeRoutes[routeIdxJ]];
        const combinedPayload = combinedNodes.reduce((acc, curr) => acc + getPayloadTons(curr), 0);

        if (combinedPayload <= maxVehiclePayload) {
          activeRoutes[routeIdxI] = combinedNodes;
          activeRoutes.splice(routeIdxJ, 1);
        }
      }
    }

    // Assign vehicles from tactical fleet
    const sortedFleet = [...this.fleet].sort((a, b) => b.maxPayloadTons - a.maxPayloadTons);
    const assignedRoutes: ConvoyRoute[] = [];
    const unassignedNodes: TheaterNode[] = [];

    for (let rIdx = 0; rIdx < activeRoutes.length; rIdx++) {
      const routeNodes = activeRoutes[rIdx];
      const routePayload = routeNodes.reduce((acc, curr) => acc + getPayloadTons(curr), 0);
      const vehicle = sortedFleet[rIdx % sortedFleet.length];

      if (!vehicle || routePayload > vehicle.maxPayloadTons * 1.1) {
        unassignedNodes.push(...routeNodes);
        continue;
      }

      // 2-Opt local refinement on the stop sequence
      const optimizedNodes = this.optimize2Opt([this.depot, ...routeNodes, this.depot]);
      const stops = optimizedNodes.slice(1, -1); // remove depot ends

      // Build route legs with A* pathfinding
      const legs: ConvoyLeg[] = [];
      let totalDist = 0;
      let totalTransitMin = 0;
      let totalFuelBurn = 0;
      let maxRisk = 0;

      const fullSequence = [this.depot, ...stops, this.depot];
      for (let s = 0; s < fullSequence.length - 1; s++) {
        const from = fullSequence[s];
        const to = fullSequence[s + 1];
        const pathResult = this.pathfinder.findPath(from, to, this.threatZones);

        const legTransitTime = (pathResult.distanceKm / vehicle.avgSpeedKph) * 60;
        const legFuelBurn = pathResult.distanceKm * vehicle.fuelBurnRateGalPerKm;

        legs.push({
          fromNodeId: from.id,
          toNodeId: to.id,
          distanceKm: pathResult.distanceKm,
          transitTimeMin: legTransitTime,
          fuelBurnedGal: legFuelBurn,
          accumulatedRisk: pathResult.threatExposure,
          waypoints: pathResult.path,
        });

        totalDist += pathResult.distanceKm;
        totalTransitMin += legTransitTime;
        totalFuelBurn += legFuelBurn;
        maxRisk = Math.max(maxRisk, pathResult.threatExposure);
      }

      const deliveredSupplies = stops.reduce(
        (acc, curr) => ({
          classIII_fuel_gal: acc.classIII_fuel_gal + curr.demands.classIII_fuel_gal,
          classV_ammo_tons: acc.classV_ammo_tons + curr.demands.classV_ammo_tons,
          classIX_spares_kg: acc.classIX_spares_kg + curr.demands.classIX_spares_kg,
        }),
        { classIII_fuel_gal: 0, classV_ammo_tons: 0, classIX_spares_kg: 0 }
      );

      assignedRoutes.push({
        vehicleId: vehicle.id,
        vehicleModel: vehicle.model,
        assignedNodes: stops,
        legs,
        totalDistanceKm: totalDist,
        totalTransitTimeMin: totalTransitMin,
        totalFuelBurnedGal: totalFuelBurn,
        maxLegRisk: maxRisk,
        deliveredSupplies,
        feasible: totalDist <= vehicle.operationalRangeKm,
      });
    }

    const totalFleetDist = assignedRoutes.reduce((acc, r) => acc + r.totalDistanceKm, 0);
    const totalFleetFuel = assignedRoutes.reduce((acc, r) => acc + r.totalFuelBurnedGal, 0);
    const totalFleetRisk = assignedRoutes.reduce((acc, r) => acc + r.maxLegRisk, 0) / (assignedRoutes.length || 1);

    return {
      depotId: this.depot.id,
      routes: assignedRoutes,
      totalFleetDistanceKm: totalFleetDist,
      totalTheaterFuelBurnGal: totalFleetFuel,
      totalTheaterRiskIndex: totalFleetRisk,
      unassignedNodes,
      computationTimeMs: performance.now() - startTime,
    };
  }

  /**
   * 2-Opt local search refinement to eliminate intersecting tour segments
   */
  public optimize2Opt(route: TheaterNode[]): TheaterNode[] {
    if (route.length <= 3) return route;

    let bestRoute = [...route];
    let improved = true;
    let iteration = 0;
    const max2OptIter = 50;

    const calcRouteDist = (r: TheaterNode[]): number => {
      let d = 0;
      for (let i = 0; i < r.length - 1; i++) {
        d += this.getDistance(r[i], r[i + 1]);
      }
      return d;
    };

    let bestDist = calcRouteDist(bestRoute);

    while (improved && iteration++ < max2OptIter) {
      improved = false;
      for (let i = 1; i < bestRoute.length - 2; i++) {
        for (let j = i + 1; j < bestRoute.length - 1; j++) {
          // Reverse segment between i and j
          const newRoute = [
            ...bestRoute.slice(0, i),
            ...bestRoute.slice(i, j + 1).reverse(),
            ...bestRoute.slice(j + 1),
          ];

          const newDist = calcRouteDist(newRoute);
          if (newDist < bestDist - 1e-4) {
            bestDist = newDist;
            bestRoute = newRoute;
            improved = true;
            break;
          }
        }
        if (improved) break;
      }
    }

    return bestRoute;
  }
}
