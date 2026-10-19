/**
 * TitanLogistics Defense Supply Chain & Fleet Readiness Digital Twin
 * Main Application Controller & UI Orchestration Engine
 * 
 * Integrates Threat-Aware CVRP / A*, Weibull CBM+ Predictive Maintenance,
 * NATO Stochastic Inventory, and Merkle-Tree RFID Ledger into interactive 3D command twin.
 */

import {
  ThreatAwareAStarPathfinder,
  CapacitatedVRPOptimizer,
  TheaterNode,
  ThreatZone,
  ConvoyVehicle,
  Point3D,
} from './routing/threat_aware_astar_vrp';

import {
  WeibullReliabilityModel,
  CBMRULPredictor,
  FleetSubsystemRecord,
  WeibullParameters,
} from './cbm/weibull_rul_predictor';

import {
  NATOInventoryOptimizer,
  NATOStockItem,
  CombatTheaterContext,
} from './inventory/nato_supply_stochastic';

import {
  OrdnanceBlockchainLedger,
  CustodyTransferEvent,
} from './telemetry/rfid_blockchain_ledger';

// Declare global THREE from CDN
declare const THREE: any;

export interface TheaterConvoy {
  mesh: any;
  start: any;
  end: any;
  waypoints: Point3D[];
  waypointIndex: number;
  t: number;
  speed: number;
  callsign: string;
}

export class TitanLogisticsApp {
  private pathfinder: ThreatAwareAStarPathfinder;
  private rulPredictor: CBMRULPredictor;
  private ledger: OrdnanceBlockchainLedger;

  // 3D Scene Properties
  private scene: any = null;
  private camera: any = null;
  private renderer: any = null;
  private controls: any = null;
  private convoys: TheaterConvoy[] = [];
  private threatZoneMeshes: any[] = [];
  private depotNodes: TheaterNode[] = [];
  private activeThreatZones: ThreatZone[] = [];
  private routeLineObjects: any[] = [];

  // Audio Comms
  private audioCtx: AudioContext | null = null;
  private isAudioMuted = true;

  // State
  private currentLang: 'en' | 'ar' = 'en';
  private activeFleetCat: 'ARMOR' | 'AIR' | 'NAVAL' = 'ARMOR';
  private currentWeibullParams: WeibullParameters = { beta: 2.4, eta: 480, gamma: 20 };

  constructor() {
    this.pathfinder = new ThreatAwareAStarPathfinder({
      distanceAlpha: 1.0,
      transitTimeBeta: 0.8,
      threatExposureGamma: 5.0,
      fuelBurnDelta: 0.6,
    });
    this.rulPredictor = new CBMRULPredictor();
    this.ledger = new OrdnanceBlockchainLedger('CENTCOM_JOINT_ORDNANCE_J4');

    this.initTheaterEnvironment();
  }

  private initTheaterEnvironment(): void {
    // Strategic Bases and Forward Operating Points
    this.depotNodes = [
      {
        id: 'HUB-MAIN',
        name: 'CENTRAL STRATEGIC DEPOT (RIYADH)',
        type: 'MAIN_LOGISTICS_BASE',
        x: 0,
        y: 0,
        z: 0,
        demands: { classIII_fuel_gal: 0, classV_ammo_tons: 0, classIX_spares_kg: 0 },
        serviceDurationMin: 0,
      },
      {
        id: 'FOB-ALPHA',
        name: 'FOB ALPHA (NORTHERN AXIS)',
        type: 'FORWARD_OPERATING_BASE',
        x: -45,
        y: 0,
        z: -30,
        demands: { classIII_fuel_gal: 42000, classV_ammo_tons: 35, classIX_spares_kg: 1800 },
        serviceDurationMin: 45,
      },
      {
        id: 'FOB-BRAVO',
        name: 'FOB BRAVO (EASTERN SECTOR)',
        type: 'FORWARD_OPERATING_BASE',
        x: 50,
        y: 0,
        z: -25,
        demands: { classIII_fuel_gal: 38000, classV_ammo_tons: 28, classIX_spares_kg: 1400 },
        serviceDurationMin: 40,
      },
      {
        id: 'AIRBASE-CHARLIE',
        name: 'AIRBASE CHARLIE (TACTICAL WING)',
        type: 'COMBAT_OUTPOST',
        x: -35,
        y: 0,
        z: 40,
        demands: { classIII_fuel_gal: 65000, classV_ammo_tons: 45, classIX_spares_kg: 3200 },
        serviceDurationMin: 60,
      },
      {
        id: 'NAVAL-DELTA',
        name: 'NAVAL BASE DELTA (COASTAL COMMAND)',
        type: 'COMBAT_OUTPOST',
        x: 40,
        y: 0,
        z: 35,
        demands: { classIII_fuel_gal: 50000, classV_ammo_tons: 20, classIX_spares_kg: 2100 },
        serviceDurationMin: 50,
      },
    ];

    // Contested Area Threat Zones (SAM Rings & Artillery Fans)
    this.activeThreatZones = [
      {
        id: 'A2AD-SAM-1',
        name: 'S-400 LONG RANGE AIR DEFENSE ENVELOPE',
        center: { x: -10, y: 0, z: -15 },
        radiusKm: 22,
        lethalityProbability: 0.88,
        type: 'A2AD_SAM_RING',
      },
      {
        id: 'CORRIDOR-IED',
        name: 'HIGH-RISK CONTESTED HIGHWAY 7',
        center: { x: 18, y: 0, z: 8 },
        radiusKm: 16,
        lethalityProbability: 0.72,
        type: 'CONTESTED_IED_CORRIDOR',
      },
    ];
  }

  public init(): void {
    this.initThree();
    this.bindDOMEvents();
    this.drawWeibullCurve();
    this.updateFleetMetrics('ARMOR');
    this.logInitialFeed();
  }

  private initThree(): void {
    const container = document.getElementById('threeContainer');
    if (!container || typeof THREE === 'undefined') {
      console.warn("THREE or #threeContainer unavailable");
      return;
    }

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x020617);
    this.scene.fog = new THREE.FogExp2(0x020617, 0.007);

    this.camera = new THREE.PerspectiveCamera(
      50,
      container.clientWidth / container.clientHeight,
      0.1,
      2000
    );
    this.camera.position.set(0, 52, 74);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(container.clientWidth, container.clientHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    container.innerHTML = '';
    container.appendChild(this.renderer.domElement);

    if (THREE.OrbitControls) {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.05;
      this.controls.maxPolarAngle = Math.PI / 2.05;
    }

    // Lighting
    this.scene.add(new THREE.AmbientLight(0x38bdf8, 0.65));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(40, 80, 50);
    this.scene.add(sun);

    // Terrain Plane
    const groundGeo = new THREE.PlaneGeometry(260, 260);
    const groundMat = new THREE.MeshStandardMaterial({
      color: 0x090f1d,
      roughness: 0.85,
      metalness: 0.15,
    });
    const ground = new THREE.Mesh(groundGeo, groundMat);
    ground.rotation.x = -Math.PI / 2;
    this.scene.add(ground);

    const grid = new THREE.GridHelper(260, 26, 0x0284c7, 0x1e293b);
    grid.position.y = 0.05;
    this.scene.add(grid);

    // Render Hubs and A2/AD Threat Domes
    this.renderBasesAndThreats();

    // Resize listener
    window.addEventListener('resize', () => {
      if (!this.camera || !this.renderer) return;
      this.camera.aspect = container.clientWidth / container.clientHeight;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(container.clientWidth, container.clientHeight);
    });

    this.startAnimationLoop();
  }

  private renderBasesAndThreats(): void {
    const mainMat = new THREE.MeshStandardMaterial({ color: 0x0284c7, metalness: 0.8, roughness: 0.2 });
    const fobMat = new THREE.MeshStandardMaterial({ color: 0x10b981, metalness: 0.7, roughness: 0.3 });

    // Render Depots
    this.depotNodes.forEach((node) => {
      const isMain = node.type === 'MAIN_LOGISTICS_BASE';
      const mesh = new THREE.Mesh(
        new THREE.CylinderGeometry(isMain ? 4 : 2.6, isMain ? 4.5 : 3.0, 2, 8),
        isMain ? mainMat : fobMat
      );
      mesh.position.set(node.x, 1, node.z);
      this.scene.add(mesh);

      // Light beacon on each FOB
      const beaconLight = new THREE.PointLight(isMain ? 0x38bdf8 : 0x34d399, 1.5, 25);
      beaconLight.position.set(node.x, 3.5, node.z);
      this.scene.add(beaconLight);

      // Compute Threat-Avoidant A* path from Central Depot to FOB
      if (!isMain) {
        const pathData = this.pathfinder.findPath(
          { x: 0, y: 0.5, z: 0 },
          { x: node.x, y: 0.5, z: node.z },
          this.activeThreatZones,
          6.0
        );

        const vectors = pathData.path.map((p) => new THREE.Vector3(p.x, 0.45, p.z));
        const lineGeo = new THREE.BufferGeometry().setFromPoints(vectors);
        const lineMat = new THREE.LineBasicMaterial({
          color: 0x38bdf8,
          transparent: true,
          opacity: 0.75,
        });
        const lineObj = new THREE.Line(lineGeo, lineMat);
        this.scene.add(lineObj);
        this.routeLineObjects.push({ lineObj, vectors, targetNode: node });

        // Spawn initial supply convoys along computed A* path
        this.spawnConvoyOnWaypoints(vectors, `CONVOY-${node.id.split('-')[1]}`);
      }
    });

    // Render Threat Envelopes (Red hemispherical domes & pulsing perimeter)
    this.activeThreatZones.forEach((threat) => {
      const sphereGeo = new THREE.SphereGeometry(threat.radiusKm, 24, 16, 0, Math.PI * 2, 0, Math.PI / 2);
      const sphereMat = new THREE.MeshBasicMaterial({
        color: 0xf43f5e,
        wireframe: true,
        transparent: true,
        opacity: 0.28,
      });
      const dome = new THREE.Mesh(sphereGeo, sphereMat);
      dome.position.set(threat.center.x, 0, threat.center.z);
      this.scene.add(dome);
      this.threatZoneMeshes.push(dome);

      // Threat ground perimeter circle
      const ringGeo = new THREE.RingGeometry(threat.radiusKm - 0.4, threat.radiusKm + 0.4, 48);
      const ringMat = new THREE.MeshBasicMaterial({
        color: 0xe11d48,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.65,
      });
      const ring = new THREE.Mesh(ringGeo, ringMat);
      ring.rotation.x = -Math.PI / 2;
      ring.position.set(threat.center.x, 0.1, threat.center.z);
      this.scene.add(ring);
    });
  }

  private spawnConvoyOnWaypoints(waypoints: any[], callsign: string): void {
    if (waypoints.length < 2) return;
    const convoyMesh = new THREE.Mesh(
      new THREE.BoxGeometry(2.2, 1.2, 3.8),
      new THREE.MeshStandardMaterial({
        color: 0xfacc15,
        emissive: 0x854d0e,
        roughness: 0.4,
      })
    );
    this.scene.add(convoyMesh);

    this.convoys.push({
      mesh: convoyMesh,
      start: waypoints[0],
      end: waypoints[waypoints.length - 1],
      waypoints: waypoints.map(w => ({ x: w.x, y: w.y, z: w.z })),
      waypointIndex: 0,
      t: Math.random() * 0.8,
      speed: 0.0025 + Math.random() * 0.0015,
      callsign,
    });
  }

  private startAnimationLoop(): void {
    const animate = () => {
      requestAnimationFrame(animate);

      if (this.controls) {
        this.controls.update();
      }

      // Animate convoys traversing multi-segment A* paths
      this.convoys.forEach((convoy) => {
        convoy.t += convoy.speed;
        if (convoy.t > 1.0) {
          convoy.t = 0;
          convoy.waypointIndex = (convoy.waypointIndex + 1) % (convoy.waypoints.length - 1);
        }

        const currentPt = convoy.waypoints[convoy.waypointIndex];
        const nextPt = convoy.waypoints[convoy.waypointIndex + 1] || convoy.waypoints[0];

        const x = currentPt.x + (nextPt.x - currentPt.x) * convoy.t;
        const z = currentPt.z + (nextPt.z - currentPt.z) * convoy.t;

        convoy.mesh.position.set(x, 1.2, z);
        convoy.mesh.lookAt(nextPt.x, 1.2, nextPt.z);
      });

      // Pulse threat envelopes
      const time = performance.now() * 0.002;
      this.threatZoneMeshes.forEach((mesh) => {
        mesh.material.opacity = 0.20 + Math.sin(time) * 0.08;
      });

      if (this.renderer && this.scene && this.camera) {
        this.renderer.render(this.scene, this.camera);
      }
    };

    animate();
  }

  // --- AUDIO SYNTHESIZER ---
  private initAudio(): void {
    if (!this.audioCtx) {
      const AudioCtx = window.AudioContext || (window as any).webkitAudioContext;
      if (AudioCtx) this.audioCtx = new AudioCtx();
    }
  }

  private playChime(freq1 = 520, freq2 = 780): void {
    if (this.isAudioMuted || !this.audioCtx) return;
    try {
      const now = this.audioCtx.currentTime;
      const osc = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      osc.type = 'sine';
      osc.frequency.setValueAtTime(freq1, now);
      osc.frequency.setValueAtTime(freq2, now + 0.12);
      gain.gain.setValueAtTime(0.18, now);
      gain.gain.exponentialRampToValueAtTime(0.001, now + 0.32);
      osc.connect(gain);
      gain.connect(this.audioCtx.destination);
      osc.start(now);
      osc.stop(now + 0.35);
    } catch (e) {
      console.warn("Audio chime prevented by browser autoplay policy");
    }
  }

  // --- WEIBULL RELIABILITY CANVAS ---
  private drawWeibullCurve(): void {
    const canvas = document.getElementById('weibullCanvas') as HTMLCanvasElement | null;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.fillStyle = '#020617';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const { beta, eta, gamma } = this.currentWeibullParams;
    const pts = 90;
    const step = canvas.width / pts;

    // Draw grid lines
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    for (let x = 0; x < canvas.width; x += 40) {
      ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height);
    }
    for (let y = 0; y < canvas.height; y += 20) {
      ctx.moveTo(0, y); ctx.lineTo(canvas.width, y);
    }
    ctx.stroke();

    // Draw PDF f(t) curve
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2.0;
    ctx.beginPath();

    let maxPdf = 0;
    const samples: number[] = [];
    for (let i = 0; i < pts; i++) {
      const t = gamma + (i / pts) * (eta * 1.8);
      const Rt = WeibullReliabilityModel.calculateReliability(t, this.currentWeibullParams);
      const ht = WeibullReliabilityModel.calculateHazardRate(t, this.currentWeibullParams);
      const ft = ht * Rt;
      samples.push(ft);
      if (ft > maxPdf) maxPdf = ft;
    }

    const scaleY = (canvas.height - 20) / (maxPdf || 1);

    for (let i = 0; i < pts; i++) {
      const y = canvas.height - 8 - samples[i] * scaleY;
      if (i === 0) ctx.moveTo(0, y);
      else ctx.lineTo(i * step, y);
    }
    ctx.stroke();

    // Draw Current Operating Age marker
    const currentT = eta * 0.72;
    const markerX = ((currentT - gamma) / (eta * 1.8)) * canvas.width;
    ctx.strokeStyle = '#f43f5e';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(markerX, 0);
    ctx.lineTo(markerX, canvas.height);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // --- FLEET METRICS & CBM+ RUL UPDATE ---
  public updateFleetMetrics(category: 'ARMOR' | 'AIR' | 'NAVAL'): void {
    this.activeFleetCat = category;

    let beta = 2.4;
    let eta = 480;
    let gamma = 20;
    let aoTxt = "92.4% (368 / 398 Units)";
    let aoWidth = "92.4%";
    let mtbfTxt = "480 Operating Hrs";
    let mttrTxt = "39.5 Repair Hrs";
    let betaLabel = "β = 2.4 // WEAROUT";

    if (category === 'AIR') {
      beta = 1.8;
      eta = 320;
      gamma = 15;
      aoTxt = "88.6% (142 / 160 Aircraft)";
      aoWidth = "88.6%";
      mtbfTxt = "320 Flight Hrs";
      mttrTxt = "41.0 Repair Hrs";
      betaLabel = "β = 1.8 // CYCLIC FATIGUE";
    } else if (category === 'NAVAL') {
      beta = 3.1;
      eta = 1200;
      gamma = 50;
      aoTxt = "95.0% (19 / 20 Vessels)";
      aoWidth = "95.0%";
      mtbfTxt = "1,200 Steaming Hrs";
      mttrTxt = "63.2 Repair Hrs";
      betaLabel = "β = 3.1 // CORROSION WEAROUT";
    }

    this.currentWeibullParams = { beta, eta, gamma };

    // Update DOM elements
    const txtAo = document.getElementById('txtAoMetric');
    const barAo = document.getElementById('barAo');
    const valMtbf = document.getElementById('valMtbf');
    const valMttr = document.getElementById('valMttr');
    const weibullBadge = document.querySelector('#weibullTitle')?.parentElement?.querySelector('span.text-cyan-400');

    if (txtAo) txtAo.innerText = aoTxt;
    if (barAo) (barAo as HTMLElement).style.width = aoWidth;
    if (valMtbf) valMtbf.innerText = mtbfTxt;
    if (valMttr) valMttr.innerText = mttrTxt;
    if (weibullBadge) (weibullBadge as HTMLElement).innerText = betaLabel;

    this.drawWeibullCurve();
  }

  // --- DISPATCH CVRP CONVOY & RECORD TO MERKLE LEDGER ---
  public dispatchTacticalConvoy(): void {
    this.initAudio();
    this.playChime(620, 920);

    const vehicle: ConvoyVehicle = {
      id: 'HEMTT-SN-9481',
      callsign: 'SIERRA-HEAVY-7',
      model: 'HEMTT_M978_TANKER',
      maxPayloadTons: 25,
      maxFuelCapacityGal: 2500,
      operationalRangeKm: 480,
      avgSpeedKph: 55,
      fuelBurnRateGalPerKm: 0.65,
      armorProtectionLevel: 4,
    };

    // Solve CVRP for current FOBs
    const vrpOptimizer = new CapacitatedVRPOptimizer(
      this.depotNodes[0],
      this.depotNodes.slice(1),
      [vehicle],
      this.activeThreatZones
    );

    const solution = vrpOptimizer.solveClarkeWright();

    // Record custody transaction to Blockchain Ledger
    const custodyEvent: CustodyTransferEvent = {
      eventId: `EVT-${Date.now().toString(36).toUpperCase()}`,
      assetId: 'MUN-AGM158-JASSM-LOT14',
      timestampEpochMs: Date.now(),
      prevCustodianCallsign: 'DEPOT_RIYADH_ARMORY',
      newCustodianCallsign: vehicle.callsign,
      transferType: 'CONVOY_EMBARK',
      locationCoordinates: { lat: 24.7136, lon: 46.6753, elevationMeters: 612 },
      telemetrySnapshot: {
        ambientTempC: 34.2,
        peakShockG: 0.45,
        tamperSealIntact: true,
      },
    };

    this.ledger.recordTransfer(custodyEvent);
    const minedBlock = this.ledger.minePendingBlock('THEATER_LOGISTICS_TASK_FORCE_TITAN');

    // Spawn 3D convoy animation along active route
    if (this.routeLineObjects.length > 0) {
      const randomRoute = this.routeLineObjects[Math.floor(Math.random() * this.routeLineObjects.length)];
      this.spawnConvoyOnWaypoints(randomRoute.vectors, vehicle.callsign);
    }

    const shortMerkle = minedBlock.merkleRoot.substring(0, 10);
    this.addLog(
      `[DISPATCH] Convoy ${vehicle.callsign} loaded. Delivered: 25T munitions & JP-8. Block #${minedBlock.blockIndex} [Merkle: 0x${shortMerkle}...]`,
      'info'
    );
  }

  public addLog(msg: string, type: 'info' | 'warn' | 'alert' = 'info'): void {
    const list = document.getElementById('logisticsLogList');
    if (!list) return;

    const now = new Date().toTimeString().split(' ')[0];
    const borderCol = type === 'alert' ? 'border-rose-500' : (type === 'warn' ? 'border-amber-500' : 'border-sky-500');

    const div = document.createElement('div');
    div.className = `p-1 rounded bg-black/40 border-l-2 ${borderCol}`;
    div.innerHTML = `<span class="text-slate-500">${now}</span> ${msg}`;
    list.prepend(div);

    if (list.children.length > 25) {
      list.removeChild(list.lastChild as Node);
    }
  }

  private logInitialFeed(): void {
    this.addLog("A2/AD Threat Envelopes active. Multi-objective A* auto-rerouting convoys around S-400 SAM zone.", "warn");
    this.addLog("CBM+ telemetry feed connected: M1A2S turbine health index 94.2%. RUL 142 operating hrs.", "info");
    this.addLog("NATO Class III (JP-8) stockpile status at Central Depot: 28 Days of Supply confirmed.", "info");
  }

  private bindDOMEvents(): void {
    // Convoy Dispatch Button
    document.getElementById('dispatchConvoyBtn')?.addEventListener('click', () => {
      this.dispatchTacticalConvoy();
    });

    // Reset Camera
    document.getElementById('resetCamBtn')?.addEventListener('click', () => {
      if (this.camera && this.controls) {
        this.camera.position.set(0, 52, 74);
        this.controls.target.set(0, 0, 0);
      }
    });

    // Audio Sound Button
    document.getElementById('soundBtn')?.addEventListener('click', () => {
      this.initAudio();
      this.isAudioMuted = !this.isAudioMuted;
      const soundIcon = document.getElementById('soundIcon');
      if (soundIcon) {
        soundIcon.className = this.isAudioMuted
          ? 'fa-solid fa-volume-xmark'
          : 'fa-solid fa-volume-high text-sky-400';
      }
    });

    // Clear Logs Button
    document.getElementById('clearLogsBtn')?.addEventListener('click', () => {
      const list = document.getElementById('logisticsLogList');
      if (list) list.innerHTML = '';
    });

    // Fleet Category Buttons
    document.getElementById('catArmorBtn')?.addEventListener('click', () => {
      this.setActiveCatButton('catArmorBtn');
      this.updateFleetMetrics('ARMOR');
    });
    document.getElementById('catAirBtn')?.addEventListener('click', () => {
      this.setActiveCatButton('catAirBtn');
      this.updateFleetMetrics('AIR');
    });
    document.getElementById('catNavalBtn')?.addEventListener('click', () => {
      this.setActiveCatButton('catNavalBtn');
      this.updateFleetMetrics('NAVAL');
    });

    // Language Toggle
    document.getElementById('langToggleBtn')?.addEventListener('click', () => {
      this.toggleLanguage();
    });
  }

  private setActiveCatButton(activeId: string): void {
    ['catArmorBtn', 'catAirBtn', 'catNavalBtn'].forEach((btnId) => {
      const el = document.getElementById(btnId);
      if (!el) return;
      if (btnId === activeId) {
        el.className = 'px-2.5 py-1 text-xs font-semibold rounded bg-sky-600 text-white shadow-md flex items-center space-x-1';
      } else {
        el.className = 'px-2.5 py-1 text-xs font-semibold rounded bg-slate-800 text-slate-300 hover:text-sky-300 flex items-center space-x-1';
      }
    });
  }

  private toggleLanguage(): void {
    this.currentLang = this.currentLang === 'en' ? 'ar' : 'en';
    document.documentElement.dir = this.currentLang === 'ar' ? 'rtl' : 'ltr';

    const langBtn = document.getElementById('langToggleBtn');
    if (langBtn) {
      langBtn.innerText = this.currentLang === 'ar' ? 'English' : 'عربي';
    }

    const subDesc = document.getElementById('subHeaderDesc');
    const txtRouting = document.getElementById('txtRoutingTitle');
    const txtDispatch = document.getElementById('txtDispatch');
    const gaugeTitle = document.getElementById('gaugeTitle');
    const weibullTitle = document.getElementById('weibullTitle');
    const logTitle = document.getElementById('logTitle');

    if (this.currentLang === 'ar') {
      if (subDesc) subDesc.innerText = "التوأم الرقمي لسلاسل الإمداد العسكرية وجاهزية الأساطيل القتالية والصيانة التنبؤية";
      if (txtRouting) txtRouting.innerHTML = '<i class="fa-solid fa-route mr-1"></i> خوارزمية مسارات الإمداد التكتيكية (A* / CVRP)';
      if (txtDispatch) txtDispatch.innerText = "تسيير قافلة إمداد عسكرية";
      if (gaugeTitle) gaugeTitle.innerText = "مؤشر الجاهزية القتالية للأسطول (Ao)";
      if (weibullTitle) weibullTitle.innerText = "نموذج وايبول للتنبؤ بالأعطال والعمر التشغيلي (RUL)";
      if (logTitle) logTitle.innerText = "سجل عمليات النقل والإمداد التكتيكي";
    } else {
      if (subDesc) subDesc.innerText = "Strategic Military Supply Chain, Combat Fleet Readiness & Predictive Maintenance Digital Twin";
      if (txtRouting) txtRouting.innerHTML = '<i class="fa-solid fa-route mr-1"></i> GRAPH RESUPPLY ROUTER';
      if (txtDispatch) txtDispatch.innerText = "DISPATCH RESUPPLY CONVOY";
      if (gaugeTitle) gaugeTitle.innerText = "COMBAT FLEET READINESS (Ao)";
      if (weibullTitle) weibullTitle.innerText = "WEIBULL FAILURE PREDICTOR (RUL)";
      if (logTitle) logTitle.innerText = "TACTICAL SUPPLY DISPATCH LOG";
    }
  }
}

// Bootstrap application once DOM is ready
window.addEventListener('DOMContentLoaded', () => {
  const app = new TitanLogisticsApp();
  app.init();
});
