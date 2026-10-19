/**
 * TitanLogistics Defense Supply Chain Engine
 * CBM+ (Condition-Based Maintenance Plus) & Weibull Reliability / RUL Engine
 * 
 * Provides physics-grounded reliability forecasting R(t), hazard rates h(t),
 * discrete Kalman filter degradation tracking, and Remaining Useful Life (RUL)
 * prediction for combat vehicle subsystems (turbines, transmissions, tracks).
 */

export interface WeibullParameters {
  beta: number;   // Shape parameter (slope): beta < 1 (infant mortality), beta = 1 (random), beta > 1 (wear-out)
  eta: number;    // Scale parameter (characteristic life in operating hours / cycles)
  gamma: number;  // Location / threshold parameter (failure-free operating time)
}

export interface SubsystemTelemetry {
  timestampMs: number;
  operatingHours: number;
  vibrationRmsG: number;       // Accelerometer root-mean-square in g
  oilFerrousDebrisPpm: number;  // Inductive metal sensor in engine oil
  thermalSignatureC: number;    // Subsystem core operating temp in Celsius
  acousticEmissionDb: number;   // Ultrasonic bearing stress
}

export interface KalmanFilterState {
  estimatedHealthIndex: number; // [0.0 = Failure, 1.0 = Pristine]
  degradationRatePerHour: number;
  covarianceP: number[][];      // 2x2 error covariance matrix
}

export interface RULPrediction {
  currentHealthIndex: number;
  subsystemStatus: 'NOMINAL' | 'ADVISORY' | 'CRITICAL_MAINTENANCE' | 'AOG_GROUNDED';
  estimatedRULHours: number;
  confidenceInterval95: {
    lowerHours: number;
    upperHours: number;
  };
  weibullReliabilityRt: number;
  instantaneousHazardRateHt: number;
  recommendedAction: string;
}

export interface FleetSubsystemRecord {
  assetId: string;
  subsystemName: 'GAS_TURBINE_ENGINE' | 'HYDROMECHANICAL_XMSN' | 'HYDRAULIC_ACTUATION' | 'TRACK_SUSPENSION';
  operatingHours: number;
  weibullParams: WeibullParameters;
  lastTelemetry: SubsystemTelemetry;
  kalmanState: KalmanFilterState;
}

export class WeibullReliabilityModel {
  /**
   * Approximates Gamma function Gamma(z) via Lanczos approximation (g=7, n=9)
   */
  public static gamma(z: number): number {
    if (z < 0.5) {
      return Math.PI / (Math.sin(Math.PI * z) * WeibullReliabilityModel.gamma(1 - z));
    }
    z -= 1;
    const p = [
      0.99999999999980993,
      676.5203681218851,
      -1259.1392167224028,
      771.32342877765313,
      -176.61502916214059,
      12.507343278686905,
      -0.138571095836524,
      9.9843695780195716e-6,
      1.5056327351493116e-7
    ];
    const g = 7;
    let x = p[0];
    for (let i = 1; i < p.length; i++) {
      x += p[i] / (z + i);
    }
    const t = z + g + 0.5;
    return Math.sqrt(2 * Math.PI) * Math.pow(t, z + 0.5) * Math.exp(-t) * x;
  }

  /**
   * Reliability function R(t) = P(T > t)
   * R(t) = exp( - ( (t - gamma) / eta )^beta )
   */
  public static calculateReliability(t: number, params: WeibullParameters): number {
    if (t <= params.gamma) return 1.0;
    const normalized = (t - params.gamma) / params.eta;
    return Math.exp(-Math.pow(normalized, params.beta));
  }

  /**
   * Cumulative Failure Probability F(t) = 1 - R(t)
   */
  public static calculateUnreliability(t: number, params: WeibullParameters): number {
    return 1.0 - WeibullReliabilityModel.calculateReliability(t, params);
  }

  /**
   * Instantaneous Hazard Rate h(t) = f(t) / R(t)
   * h(t) = (beta / eta) * ( (t - gamma) / eta )^(beta - 1)
   */
  public static calculateHazardRate(t: number, params: WeibullParameters): number {
    if (t <= params.gamma) return 0.0;
    const normalized = (t - params.gamma) / params.eta;
    return (params.beta / params.eta) * Math.pow(normalized, params.beta - 1);
  }

  /**
   * Mean Time Between Failures MTBF = gamma + eta * Gamma(1 + 1 / beta)
   */
  public static calculateMTBF(params: WeibullParameters): number {
    const gammaFactor = WeibullReliabilityModel.gamma(1.0 + 1.0 / params.beta);
    return params.gamma + params.eta * gammaFactor;
  }

  /**
   * Operational Availability Ao = MTBF / (MTBF + MTTR + MLDT)
   * Where MTTR is Mean Time to Repair and MLDT is Mean Logistics Delay Time
   */
  public static calculateOperationalAvailability(
    mtbfHours: number,
    mttrHours: number,
    mldtHours: number
  ): number {
    const denominator = mtbfHours + mttrHours + mldtHours;
    if (denominator <= 0) return 0;
    return mtbfHours / denominator;
  }
}

/**
 * Discrete 2-State Kalman Filter tracking Health Index (HI) and degradation drift
 */
export class DegradationKalmanFilter {
  // Process noise covariance Q
  private Q: number[][];
  // Measurement noise covariance R
  private R_variance: number;

  constructor(processNoiseVariance = 1e-5, measurementNoiseVariance = 0.04) {
    this.Q = [
      [processNoiseVariance, 0],
      [0, processNoiseVariance * 0.1],
    ];
    this.R_variance = measurementNoiseVariance;
  }

  public init(initialHI = 1.0): KalmanFilterState {
    return {
      estimatedHealthIndex: initialHI,
      degradationRatePerHour: -0.00015, // Nominal wear slope
      covarianceP: [
        [0.01, 0.0],
        [0.0, 0.0001],
      ],
    };
  }

  /**
   * Synthesizes multi-sensor telemetry into an observed scalar Health Index [0..1]
   */
  public static computeObservedHealthIndex(telemetry: SubsystemTelemetry): number {
    // Normalization bounds based on MIL-STD-810G combat vehicle vibration & thermal standards
    const normVibe = Math.min(1.0, telemetry.vibrationRmsG / 8.5);         // 8.5g severe trip threshold
    const normDebris = Math.min(1.0, telemetry.oilFerrousDebrisPpm / 150.0); // 150 ppm critical engine oil limit
    const normTemp = Math.max(0.0, Math.min(1.0, (telemetry.thermalSignatureC - 60) / 75.0)); // 60C-135C

    const compositePenalty = 0.40 * normVibe + 0.35 * normDebris + 0.25 * normTemp;
    return Math.max(0.0, Math.min(1.0, 1.0 - compositePenalty));
  }

  /**
   * Performs standard Kalman Predict-Update step across operating delta time dt (in hours)
   */
  public update(
    state: KalmanFilterState,
    observedHI: number,
    dtHours: number
  ): KalmanFilterState {
    const dt = Math.max(0.1, dtHours);

    // 1. Prediction step: x_pred = A * x
    // A = [[1, dt], [0, 1]]
    const predHI = state.estimatedHealthIndex + state.degradationRatePerHour * dt;
    const predRate = state.degradationRatePerHour;

    // P_pred = A * P * A^T + Q
    const p00 = state.covarianceP[0][0];
    const p01 = state.covarianceP[0][1];
    const p10 = state.covarianceP[1][0];
    const p11 = state.covarianceP[1][1];

    const predP00 = p00 + dt * (p10 + p01) + dt * dt * p11 + this.Q[0][0];
    const predP01 = p01 + dt * p11;
    const predP10 = p10 + dt * p11;
    const predP11 = p11 + this.Q[1][1];

    // 2. Update step with measurement z = observedHI
    // H = [1, 0]
    const innovation = observedHI - predHI;
    const S = predP00 + this.R_variance; // Innovation covariance

    // Kalman gain K = P_pred * H^T * inv(S)
    const K0 = predP00 / S;
    const K1 = predP10 / S;

    // Updated state
    const newHI = Math.max(0.0, Math.min(1.0, predHI + K0 * innovation));
    const newRate = predRate + K1 * innovation;

    // Updated covariance: P = (I - K*H) * P_pred
    const newP00 = (1.0 - K0) * predP00;
    const newP01 = (1.0 - K0) * predP01;
    const newP10 = -K1 * predP00 + predP10;
    const newP11 = -K1 * predP01 + predP11;

    return {
      estimatedHealthIndex: newHI,
      degradationRatePerHour: Math.min(-1e-6, newRate), // ensure negative wear trend
      covarianceP: [
        [newP00, newP01],
        [newP10, newP11],
      ],
    };
  }
}

/**
 * Predicts Remaining Useful Life combining Kalman health filter and Weibull hazard rate
 */
export class CBMRULPredictor {
  private kalman: DegradationKalmanFilter;
  private readonly FAILURE_THRESHOLD_HI = 0.20; // 20% Health Index indicates mandatory overhaul / mission abort

  constructor() {
    this.kalman = new DegradationKalmanFilter();
  }

  public predictSubsystemRUL(record: FleetSubsystemRecord, dtHoursSinceLast = 1.0): RULPrediction {
    const observedHI = DegradationKalmanFilter.computeObservedHealthIndex(record.lastTelemetry);
    const updatedKalman = this.kalman.update(record.kalmanState, observedHI, dtHoursSinceLast);
    record.kalmanState = updatedKalman;

    const currentHI = updatedKalman.estimatedHealthIndex;
    const degradationSlope = Math.abs(updatedKalman.degradationRatePerHour);

    // Linear extrapolation to failure threshold
    let deterministicRUL = (currentHI - this.FAILURE_THRESHOLD_HI) / Math.max(1e-5, degradationSlope);
    deterministicRUL = Math.max(0, deterministicRUL);

    // Reliability & hazard at current operational age
    const Rt = WeibullReliabilityModel.calculateReliability(record.operatingHours, record.weibullParams);
    const ht = WeibullReliabilityModel.calculateHazardRate(record.operatingHours, record.weibullParams);

    // Adjusted RUL modulated by Weibull wear-out acceleration
    const wearMultiplier = record.weibullParams.beta > 1.0
      ? Math.pow(record.operatingHours / record.weibullParams.eta, record.weibullParams.beta - 1)
      : 1.0;

    const adjustedRUL = deterministicRUL / Math.max(0.5, Math.min(3.0, wearMultiplier));

    // Standard deviation from Kalman covariance
    const hiVariance = Math.max(1e-6, updatedKalman.covarianceP[0][0]);
    const rulStdDev = Math.sqrt(hiVariance) / Math.max(1e-5, degradationSlope);

    const lowerBound = Math.max(0, adjustedRUL - 1.96 * rulStdDev);
    const upperBound = adjustedRUL + 1.96 * rulStdDev;

    // Subsystem status classification
    let status: RULPrediction['subsystemStatus'] = 'NOMINAL';
    let action = 'Continuous health monitoring active. Next scheduled inspection at H+100.';

    if (currentHI < 0.25 || adjustedRUL < 15) {
      status = 'AOG_GROUNDED';
      action = 'REDLINE WARNING: Immediate shutdown required. Subsystem exceeds catastrophic failure limit.';
    } else if (currentHI < 0.45 || adjustedRUL < 48) {
      status = 'CRITICAL_MAINTENANCE';
      action = 'Deploy field maintenance depot repair team. Expedite Class IX modular replacement.';
    } else if (currentHI < 0.70 || adjustedRUL < 120) {
      status = 'ADVISORY';
      action = 'Telemetry anomaly flagged. Pre-stage Class IX repair kit at Forward Logistics Element.';
    }

    return {
      currentHealthIndex: currentHI,
      subsystemStatus: status,
      estimatedRULHours: Math.round(adjustedRUL * 10) / 10,
      confidenceInterval95: {
        lowerHours: Math.round(lowerBound * 10) / 10,
        upperHours: Math.round(upperBound * 10) / 10,
      },
      weibullReliabilityRt: Math.round(Rt * 10000) / 10000,
      instantaneousHazardRateHt: ht,
      recommendedAction: action,
    };
  }
}
