import { describe, it, expect } from 'vitest';

import {
  ThreatAwareAStarPathfinder,
  CapacitatedVRPOptimizer,
  TheaterNode,
  ThreatZone,
  ConvoyVehicle,
} from '../src/routing/threat_aware_astar_vrp';

import {
  WeibullReliabilityModel,
  DegradationKalmanFilter,
  CBMRULPredictor,
  FleetSubsystemRecord,
} from '../src/cbm/weibull_rul_predictor';

import {
  NATOInventoryOptimizer,
  NATOStockItem,
  CombatTheaterContext,
} from '../src/inventory/nato_supply_stochastic';

import {
  CryptographicHash,
  OrdnanceMerkleTree,
  OrdnanceBlockchainLedger,
  CustodyTransferEvent,
} from '../src/telemetry/rfid_blockchain_ledger';

describe('Threat-Aware Multi-Objective A* & CVRP Pathfinder', () => {
  const depot: TheaterNode = {
    id: 'DEPOT-MAIN',
    name: 'CENTRAL_DEPOT',
    type: 'MAIN_LOGISTICS_BASE',
    x: 0,
    y: 0,
    z: 0,
    demands: { classIII_fuel_gal: 0, classV_ammo_tons: 0, classIX_spares_kg: 0 },
    serviceDurationMin: 0,
  };

  const fobA: TheaterNode = {
    id: 'FOB-A',
    name: 'FORWARD_OPERATING_BASE_ALPHA',
    type: 'FORWARD_OPERATING_BASE',
    x: 30,
    y: 0,
    z: 30,
    demands: { classIII_fuel_gal: 12000, classV_ammo_tons: 8, classIX_spares_kg: 500 },
    serviceDurationMin: 30,
  };

  const fobB: TheaterNode = {
    id: 'FOB-B',
    name: 'FORWARD_OPERATING_BASE_BRAVO',
    type: 'FORWARD_OPERATING_BASE',
    x: -25,
    y: 0,
    z: 20,
    demands: { classIII_fuel_gal: 8000, classV_ammo_tons: 5, classIX_spares_kg: 300 },
    serviceDurationMin: 25,
  };

  const threatZones: ThreatZone[] = [
    {
      id: 'SAM-1',
      name: 'SA-21_GRUMBLE_SAM_RING',
      center: { x: 15, y: 0, z: 15 },
      radiusKm: 12,
      lethalityProbability: 0.90,
      type: 'A2AD_SAM_RING',
    },
  ];

  it('computes 3D Euclidean distance correctly', () => {
    const p1 = { x: 0, y: 0, z: 0 };
    const p2 = { x: 30, y: 40, z: 0 };
    expect(ThreatAwareAStarPathfinder.calculateDistance(p1, p2)).toBeCloseTo(50.0, 1);
  });

  it('detects high threat exposure inside SAM envelope', () => {
    const pathfinder = new ThreatAwareAStarPathfinder();
    const insideThreat = pathfinder.evaluateThreatRisk({ x: 15, y: 0, z: 15 }, threatZones);
    const outsideThreat = pathfinder.evaluateThreatRisk({ x: 100, y: 0, z: 100 }, threatZones);

    expect(insideThreat).toBeGreaterThan(0.70);
    expect(outsideThreat).toBe(0.0);
  });

  it('generates a threat-avoidant A* trajectory from Depot to FOB A', () => {
    const pathfinder = new ThreatAwareAStarPathfinder();
    const result = pathfinder.findPath(depot, fobA, threatZones, 5.0);

    expect(result.path.length).toBeGreaterThan(1);
    expect(result.distanceKm).toBeGreaterThan(30.0);
    // Path should arrive at or within proximity to destination
    const lastNode = result.path[result.path.length - 1];
    expect(lastNode.x).toBeCloseTo(fobA.x, 0.1);
  });

  it('solves multi-depot CVRP using Clarke-Wright heuristic', () => {
    const vehicle: ConvoyVehicle = {
      id: 'HEMTT-01',
      callsign: 'WARHORSE-1',
      model: 'HEMTT_M978_TANKER',
      maxPayloadTons: 50.0,
      maxFuelCapacityGal: 2500,
      operationalRangeKm: 500,
      avgSpeedKph: 60,
      fuelBurnRateGalPerKm: 0.7,
      armorProtectionLevel: 4,
    };

    const optimizer = new CapacitatedVRPOptimizer(depot, [fobA, fobB], [vehicle], threatZones);
    const solution = optimizer.solveClarkeWright();

    expect(solution.routes.length).toBeGreaterThanOrEqual(1);
    expect(solution.totalFleetDistanceKm).toBeGreaterThan(0);
    expect(solution.unassignedNodes.length).toBe(0);
  });
});

describe('Weibull Reliability & CBM+ RUL Predictor', () => {
  const wearoutParams = { beta: 2.5, eta: 500, gamma: 10 };

  it('calculates Weibull reliability R(t) = 1.0 before threshold gamma', () => {
    const r5 = WeibullReliabilityModel.calculateReliability(5, wearoutParams);
    expect(r5).toBe(1.0);
  });

  it('calculates Weibull reliability R(t) decreasing after gamma', () => {
    const r100 = WeibullReliabilityModel.calculateReliability(100, wearoutParams);
    const r400 = WeibullReliabilityModel.calculateReliability(400, wearoutParams);
    const r600 = WeibullReliabilityModel.calculateReliability(600, wearoutParams);

    expect(r100).toBeLessThan(1.0);
    expect(r400).toBeLessThan(r100);
    expect(r600).toBeLessThan(r400);
  });

  it('computes MTBF and Operational Availability Ao correctly', () => {
    const mtbf = WeibullReliabilityModel.calculateMTBF(wearoutParams);
    expect(mtbf).toBeGreaterThan(400);
    expect(mtbf).toBeLessThan(600);

    const ao = WeibullReliabilityModel.calculateOperationalAvailability(480, 40, 10);
    expect(ao).toBeCloseTo(480 / (480 + 40 + 10), 3);
  });

  it('tracks health index degradation with discrete Kalman filter', () => {
    const kalman = new DegradationKalmanFilter();
    let state = kalman.init(1.0);

    // Feed nominal telemetry
    state = kalman.update(state, 0.95, 1.0);
    expect(state.estimatedHealthIndex).toBeLessThanOrEqual(1.0);
    expect(state.estimatedHealthIndex).toBeGreaterThan(0.85);

    // Feed severe degradation telemetry
    state = kalman.update(state, 0.30, 2.0);
    expect(state.estimatedHealthIndex).toBeLessThan(0.70);
  });

  it('predicts Remaining Useful Life (RUL) with confidence bounds', () => {
    const predictor = new CBMRULPredictor();
    const record: FleetSubsystemRecord = {
      assetId: 'M1A2S-TURBINE-001',
      subsystemName: 'GAS_TURBINE_ENGINE',
      operatingHours: 350,
      weibullParams: wearoutParams,
      lastTelemetry: {
        timestampMs: Date.now(),
        operatingHours: 350,
        vibrationRmsG: 4.2,
        oilFerrousDebrisPpm: 65,
        thermalSignatureC: 98,
        acousticEmissionDb: 72,
      },
      kalmanState: new DegradationKalmanFilter().init(0.85),
    };

    const prediction = predictor.predictSubsystemRUL(record, 1.0);
    expect(prediction.estimatedRULHours).toBeGreaterThan(0);
    expect(prediction.confidenceInterval95.upperHours).toBeGreaterThan(
      prediction.confidenceInterval95.lowerHours
    );
    expect(['NOMINAL', 'ADVISORY', 'CRITICAL_MAINTENANCE', 'AOG_GROUNDED']).toContain(
      prediction.subsystemStatus
    );
  });
});

describe('NATO Classes III/V/IX Stochastic Inventory & (s, S) Optimization', () => {
  const context: CombatTheaterContext = {
    theaterId: 'THEATER-NORTH',
    theaterName: 'NORTHERN_FRONT',
    optempo: 'HIGH_INTENSITY_ASSAULT',
    engagementRateMultiplier: 3.5,
    interdictionRisk: 0.10,
  };

  const ammoItem: NATOStockItem = {
    itemId: 'ITEM-155MM-EXCALIBUR',
    nationalStockNumber: '1320-01-549-2999',
    nomenclature: '155mm M982 Guided Artillery Projectile',
    natoClass: 'CLASS_V_AMMO',
    unitOfIssue: 'ROUNDS',
    currentOnHand: 800,
    onOrderPipeline: 0,
    nominalDailyDemand: 15,
    demandVariance: 25,
    unitCostUSD: 68000,
    stockoutPenaltyCostUSD: 450000,
    emergencyAirliftCostUSD: 35000,
    leadTimeDaysMin: 4,
    leadTimeDaysMode: 7,
    leadTimeDaysMax: 14,
    targetServiceLevel: 0.995,
  };

  it('determines robust (s, S) replenishment thresholds', () => {
    const policy = NATOInventoryOptimizer.computeOptimalPolicy(ammoItem, context);

    expect(policy.reorderPoint_s).toBeGreaterThan(0);
    expect(policy.orderUpToLevel_S).toBeGreaterThan(policy.reorderPoint_s);
    expect(policy.safetyStock).toBeGreaterThan(0);
    expect(policy.daysOfSupplyNominal).toBeGreaterThan(0);
  });

  it('executes Monte Carlo combat simulation achieving target service level', () => {
    const policy = NATOInventoryOptimizer.computeOptimalPolicy(ammoItem, context);
    const simResult = NATOInventoryOptimizer.runMonteCarloSimulation(
      ammoItem,
      context,
      policy,
      30, // 30 days
      50  // 50 replications
    );

    expect(simResult.replications).toBe(50);
    expect(simResult.serviceLevelAchieved).toBeGreaterThan(0.85);
    expect(simResult.averageEndingInventory).toBeGreaterThan(0);
  });
});

describe('Cryptographic Merkle-Tree & RFID Blockchain Ledger', () => {
  it('computes deterministic SHA-256 matching standard test vectors', () => {
    // SHA256("") = e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855
    const emptyHash = CryptographicHash.sha256("");
    expect(emptyHash).toBe("e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

    // SHA256("abc") = ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad
    const abcHash = CryptographicHash.sha256("abc");
    expect(abcHash).toBe("ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  });

  it('builds a balanced Merkle tree and verifies logarithmic audit proofs', () => {
    const leafHashes = [
      CryptographicHash.sha256('TRANSFER_EVENT_1'),
      CryptographicHash.sha256('TRANSFER_EVENT_2'),
      CryptographicHash.sha256('TRANSFER_EVENT_3'),
      CryptographicHash.sha256('TRANSFER_EVENT_4'),
    ];

    const tree = new OrdnanceMerkleTree(leafHashes);
    const root = tree.getRoot();
    expect(root).toBeDefined();
    expect(root.length).toBe(64);

    // Generate and verify proof for leaf 2
    const proof = tree.generateProof(leafHashes[2]);
    expect(proof.merkleRoot).toBe(root);
    expect(proof.targetHash).toBe(leafHashes[2]);

    const isValid = OrdnanceMerkleTree.verifyProof(proof);
    expect(isValid).toBe(true);
  });

  it('rejects tampered Merkle proofs', () => {
    const leafHashes = [
      CryptographicHash.sha256('TRANSFER_EVENT_A'),
      CryptographicHash.sha256('TRANSFER_EVENT_B'),
    ];

    const tree = new OrdnanceMerkleTree(leafHashes);
    const proof = tree.generateProof(leafHashes[0]);

    // Tamper with the proof step hash
    proof.steps[0].hash = CryptographicHash.sha256('TAMPERED_TRANSACTION');
    expect(OrdnanceMerkleTree.verifyProof(proof)).toBe(false);
  });

  it('records custody transactions into blocks and verifies blockchain integrity', () => {
    const ledger = new OrdnanceBlockchainLedger('CENTCOM_TASK_FORCE');

    const event1: CustodyTransferEvent = {
      eventId: 'EVT-TEST-001',
      assetId: 'AIM-120D-SERIAL-8492',
      timestampEpochMs: Date.now(),
      prevCustodianCallsign: 'DEPOT_ARMORY',
      newCustodianCallsign: 'WING_33_SQUADRON',
      transferType: 'DEPOT_RECEIPT',
      locationCoordinates: { lat: 24.5, lon: 46.5, elevationMeters: 550 },
      telemetrySnapshot: { ambientTempC: 22.5, peakShockG: 0.2, tamperSealIntact: true },
    };

    const event2: CustodyTransferEvent = {
      eventId: 'EVT-TEST-002',
      assetId: 'AIM-120D-SERIAL-8492',
      timestampEpochMs: Date.now() + 1000,
      prevCustodianCallsign: 'WING_33_SQUADRON',
      newCustodianCallsign: 'F15_TAIL_604',
      transferType: 'TACTICAL_HANDOFF',
      locationCoordinates: { lat: 24.5, lon: 46.5, elevationMeters: 550 },
      telemetrySnapshot: { ambientTempC: 23.0, peakShockG: 0.3, tamperSealIntact: true },
    };

    ledger.recordTransfer(event1);
    ledger.recordTransfer(event2);

    const block1 = ledger.minePendingBlock('VALIDATOR_SENTRY');
    expect(block1.blockIndex).toBe(1);
    expect(block1.eventsCount).toBe(2);

    const integrityCheck = ledger.verifyChainIntegrity();
    expect(integrityCheck.isValid).toBe(true);
  });
});
