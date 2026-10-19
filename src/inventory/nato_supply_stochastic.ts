/**
 * TitanLogistics Defense Supply Chain Engine
 * NATO Classes III / V / IX Stochastic Inventory & (s, S) Combat Resupply Optimizer
 * 
 * Simulates high-intensity combat consumption surges, stochastic logistical lead times,
 * and determines optimal continuous review (s, S) inventory replenishment parameters
 * to maintain mandatory 99.5% combat readiness without theater stock exhaustion.
 */

export type NATOClass = 'CLASS_III_POL' | 'CLASS_V_AMMO' | 'CLASS_IX_SPARES';

export interface CombatTheaterContext {
  theaterId: string;
  theaterName: string;
  optempo: 'PEACETIME_STANDBY' | 'LOW_INTENSITY_PATROL' | 'HIGH_INTENSITY_ASSAULT' | 'SUSTAINED_SIEGE';
  engagementRateMultiplier: number; // 1.0 (peacetime) to 8.5 (sustained peer conflict)
  interdictionRisk: number;         // Convoy ambush probability P in [0, 1]
}

export interface NATOStockItem {
  itemId: string;
  nationalStockNumber: string; // NSN 13-digit military standard
  nomenclature: string;
  natoClass: NATOClass;
  unitOfIssue: string;         // 'GALLONS', 'ROUNDS', 'PALLETS', 'UNITS'
  currentOnHand: number;
  onOrderPipeline: number;
  nominalDailyDemand: number;
  demandVariance: number;
  unitCostUSD: number;
  stockoutPenaltyCostUSD: number; // Cost of mission failure / degraded readiness
  emergencyAirliftCostUSD: number;
  leadTimeDaysMin: number;
  leadTimeDaysMode: number;
  leadTimeDaysMax: number;
  targetServiceLevel: number;     // e.g. 0.995 (99.5%)
}

export interface StochasticPolicyParams {
  reorderPoint_s: number;
  orderUpToLevel_S: number;
  safetyStock: number;
  daysOfSupplyNominal: number;
  expectedAnnualStockouts: number;
  expectedFillRate: number;
}

export interface MonteCarloDayResult {
  day: number;
  dailyDemand: number;
  endingOnHand: number;
  orderedQuantity: number;
  deliveriesReceived: number;
  stockoutAmount: number;
}

export interface MonteCarloSimulationSummary {
  replications: number;
  simulatedDays: number;
  averageEndingInventory: number;
  stockoutProbability: number;
  serviceLevelAchieved: number;
  totalStockoutEvents: number;
  totalAirliftEmergencyExpeditions: number;
  meanLeadTimeDays: number;
  totalTheaterSustainmentCostUSD: number;
}

export class NATOInventoryOptimizer {
  /**
   * Generates standard normal random variable using Box-Muller transform
   */
  public static randomGaussian(mean = 0, stdDev = 1): number {
    let u1 = Math.random();
    let u2 = Math.random();
    while (u1 <= 1e-7) u1 = Math.random(); // avoid log(0)
    const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
    return mean + z0 * stdDev;
  }

  /**
   * Sample from Triangular distribution Triangular(a, c, b)
   */
  public static sampleTriangular(a: number, c: number, b: number): number {
    const u = Math.random();
    const fc = (c - a) / (b - a);
    if (u < fc) {
      return a + Math.sqrt(u * (b - a) * (c - a));
    }
    return b - Math.sqrt((1 - u) * (b - a) * (b - c));
  }

  /**
   * Inverse Standard Normal CDF approximation (Abramowitz and Stegun)
   */
  public static normalQuantile(p: number): number {
    if (p <= 0 || p >= 1) {
      throw new Error("Probability p must be in (0, 1)");
    }
    // Rational approximation for lower/upper tails
    const q = p < 0.5 ? p : 1.0 - p;
    const t = Math.sqrt(-2.0 * Math.log(q));
    const c0 = 2.515517;
    const c1 = 0.802853;
    const c2 = 0.010328;
    const d1 = 1.432788;
    const d2 = 0.189269;
    const d3 = 0.001308;
    const z = t - (c0 + c1 * t + c2 * t * t) / (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);
    return p < 0.5 ? -z : z;
  }

  /**
   * Calculates continuous-review (s, S) replenishment thresholds
   * based on stochastic lead-time demand distribution
   */
  public static computeOptimalPolicy(
    item: NATOStockItem,
    context: CombatTheaterContext
  ): StochasticPolicyParams {
    const multiplier = context.engagementRateMultiplier;
    const adjustedMeanDemand = item.nominalDailyDemand * multiplier;
    const adjustedStdDemand = Math.sqrt(item.demandVariance) * Math.sqrt(multiplier);

    // Stochastic lead time moments from Triangular distribution
    const a = item.leadTimeDaysMin;
    const c = item.leadTimeDaysMode;
    const b = item.leadTimeDaysMax;
    const meanLeadTime = (a + b + c) / 3.0;
    const varLeadTime = (a * a + b * b + c * c - a * b - a * c - b * c) / 18.0;
    const stdLeadTime = Math.sqrt(Math.max(1e-4, varLeadTime));

    // Lead Time Demand (LTD) variance by total variance formula:
    // Var(LTD) = E[L] * Var(D) + (E[D])^2 * Var(L)
    const varLTD = meanLeadTime * (adjustedStdDemand * adjustedStdDemand) +
                   (adjustedMeanDemand * adjustedMeanDemand) * (stdLeadTime * stdLeadTime);
    const sigmaLTD = Math.sqrt(varLTD);

    // Safety factor for target combat service level (e.g. 99.5% -> z ~ 2.576)
    const zFactor = NATOInventoryOptimizer.normalQuantile(item.targetServiceLevel);
    const safetyStock = Math.ceil(zFactor * sigmaLTD);

    // Expected demand over lead time
    const expectedLTD = adjustedMeanDemand * meanLeadTime;

    // Reorder point s = E[LTD] + Safety Stock
    const reorderPoint_s = Math.ceil(expectedLTD + safetyStock);

    // Economic Order Quantity (EOQ) heuristic adjusted for combat holding penalty
    const holdingCostAnnual = item.unitCostUSD * 0.15; // 15% annual military warehousing
    const holdingCostDaily = holdingCostAnnual / 365.0;
    const fixedOrderCost = 1500; // Administrative and military convoy setup overhead

    const eoq = Math.ceil(Math.sqrt((2 * adjustedMeanDemand * fixedOrderCost) / Math.max(0.01, holdingCostDaily)));
    const orderUpToLevel_S = reorderPoint_s + Math.max(eoq, Math.ceil(adjustedMeanDemand * 7)); // min 7 days buffer

    const daysOfSupply = adjustedMeanDemand > 0 ? reorderPoint_s / adjustedMeanDemand : 999;

    return {
      reorderPoint_s,
      orderUpToLevel_S,
      safetyStock,
      daysOfSupplyNominal: Math.round(daysOfSupply * 10) / 10,
      expectedAnnualStockouts: Math.round((1.0 - item.targetServiceLevel) * 365 * 100) / 100,
      expectedFillRate: item.targetServiceLevel,
    };
  }

  /**
   * Runs high-fidelity Monte Carlo combat simulation across mission horizon
   */
  public static runMonteCarloSimulation(
    item: NATOStockItem,
    context: CombatTheaterContext,
    policy: StochasticPolicyParams,
    simulatedDays = 90,
    replications = 500
  ): MonteCarloSimulationSummary {
    let totalStockoutsCount = 0;
    let totalEmergencyAirlifts = 0;
    let totalDemandedAcrossReps = 0;
    let totalFulfilledAcrossReps = 0;
    let totalEndingInventorySum = 0;
    let leadTimeAccumulator = 0;
    let leadTimeObservations = 0;
    let totalExpenditureUSD = 0;

    const meanDaily = item.nominalDailyDemand * context.engagementRateMultiplier;
    const stdDaily = Math.sqrt(item.demandVariance) * Math.sqrt(context.engagementRateMultiplier);

    for (let rep = 0; rep < replications; rep++) {
      let currentOnHand = item.currentOnHand;
      // Pipeline orders: { arrivalDay, quantity }
      const pipeline: Array<{ arrivalDay: number; quantity: number }> = [];

      for (let day = 1; day <= simulatedDays; day++) {
        // 1. Deliveries arriving today
        let deliveriesToday = 0;
        for (let i = pipeline.length - 1; i >= 0; i--) {
          if (pipeline[i].arrivalDay <= day) {
            deliveriesToday += pipeline[i].quantity;
            pipeline.splice(i, 1);
          }
        }
        currentOnHand += deliveriesToday;

        // 2. Stochastic combat surge demand for the day
        let demand = Math.max(0, Math.round(NATOInventoryOptimizer.randomGaussian(meanDaily, stdDaily)));
        // Combat shock spikes: 5% chance of intense artillery or armor clash
        if (Math.random() < 0.05) {
          demand *= 2.5;
        }

        totalDemandedAcrossReps += demand;

        if (currentOnHand >= demand) {
          currentOnHand -= demand;
          totalFulfilledAcrossReps += demand;
        } else {
          // Stockout event occurs
          const deficit = demand - currentOnHand;
          totalFulfilledAcrossReps += currentOnHand;
          currentOnHand = 0;
          totalStockoutsCount++;
          totalExpenditureUSD += deficit * item.stockoutPenaltyCostUSD;

          // Dispatch emergency tactical airlift
          totalEmergencyAirlifts++;
          totalExpenditureUSD += item.emergencyAirliftCostUSD;
          // Emergency airlift arrives in 1 day
          pipeline.push({ arrivalDay: day + 1, quantity: deficit });
        }

        totalEndingInventorySum += currentOnHand;

        // 3. Periodic review replenishment check
        const totalInventoryPosition = currentOnHand + pipeline.reduce((sum, o) => sum + o.quantity, 0);

        if (totalInventoryPosition <= policy.reorderPoint_s) {
          const orderQty = policy.orderUpToLevel_S - totalInventoryPosition;
          if (orderQty > 0) {
            // Sample stochastic lead time
            let leadTime = Math.round(
              NATOInventoryOptimizer.sampleTriangular(
                item.leadTimeDaysMin,
                item.leadTimeDaysMode,
                item.leadTimeDaysMax
              )
            );

            // Risk of enemy interdiction delaying convoy
            if (Math.random() < context.interdictionRisk) {
              leadTime += Math.round(2 + Math.random() * 5); // +2 to 7 days delay
            }

            leadTimeAccumulator += leadTime;
            leadTimeObservations++;
            pipeline.push({ arrivalDay: day + leadTime, quantity: orderQty });
            totalExpenditureUSD += orderQty * item.unitCostUSD;
          }
        }
      }
    }

    const totalDaysSimulated = replications * simulatedDays;
    const avgEndingInventory = totalEndingInventorySum / totalDaysSimulated;
    const fillRate = totalDemandedAcrossReps > 0
      ? totalFulfilledAcrossReps / totalDemandedAcrossReps
      : 1.0;

    return {
      replications,
      simulatedDays,
      averageEndingInventory: Math.round(avgEndingInventory),
      stockoutProbability: totalStockoutsCount / totalDaysSimulated,
      serviceLevelAchieved: Math.round(fillRate * 10000) / 10000,
      totalStockoutEvents: totalStockoutsCount,
      totalAirliftEmergencyExpeditions: totalEmergencyAirlifts,
      meanLeadTimeDays: leadTimeObservations > 0 ? Math.round((leadTimeAccumulator / leadTimeObservations) * 10) / 10 : item.leadTimeDaysMode,
      totalTheaterSustainmentCostUSD: Math.round(totalExpenditureUSD / replications),
    };
  }
}
