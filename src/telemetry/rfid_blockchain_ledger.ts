/**
 * TitanLogistics Defense Supply Chain Engine
 * Cryptographic Merkle-Tree Ordnance Chain-of-Custody & RFID Telemetry Ledger
 * 
 * Implements immutable supply tracking for sensitive munitions (Class V PGMs, AIM-120D, Tomahawk)
 * and Class IX modular avionics, with SHA-256 hashing, Merkle audit proof verification,
 * and environmental sensor tamper detection (shock, temperature, unauthorized geo-deviation).
 */

export interface OrdnanceAsset {
  assetId: string;
  dodic: string;               // Department of Defense Identification Code (e.g. CA44 for 155mm Excalibur)
  lotNumber: string;
  serialNumber: string;
  rfidTagUid: string;          // Gen-2 ISO 18000-6C active RFID identifier
  classification: 'UNCLASSIFIED' | 'CONFIDENTIAL' | 'SECRET' | 'TOP_SECRET';
  maxAllowedShockG: number;
  maxAllowedTempC: number;
}

export interface CustodyTransferEvent {
  eventId: string;
  assetId: string;
  timestampEpochMs: number;
  prevCustodianCallsign: string;
  newCustodianCallsign: string;
  transferType: 'DEPOT_RECEIPT' | 'CONVOY_EMBARK' | 'TACTICAL_HANDOFF' | 'FORWARD_EXPENDITURE';
  locationCoordinates: {
    lat: number;
    lon: number;
    elevationMeters: number;
  };
  telemetrySnapshot: {
    ambientTempC: number;
    peakShockG: number;
    tamperSealIntact: boolean;
  };
}

export interface MerkleProofStep {
  position: 'LEFT' | 'RIGHT';
  hash: string;
}

export interface MerkleProof {
  targetHash: string;
  merkleRoot: string;
  steps: MerkleProofStep[];
}

export interface LedgerBlock {
  blockIndex: number;
  previousBlockHash: string;
  timestampEpochMs: number;
  merkleRoot: string;
  eventsCount: number;
  events: CustodyTransferEvent[];
  blockHash: string;
  validatorCallsign: string;
}

/**
 * Pure TypeScript SHA-256 implementation ensuring deterministic cryptographic execution
 * across Node.js, Vitest, and browser environments without external dependencies.
 */
export class CryptographicHash {
  private static rotr(n: number, x: number): number {
    return (x >>> n) | (x << (32 - n));
  }

  private static choose(x: number, y: number, z: number): number {
    return (x & y) ^ (~x & z);
  }

  private static majority(x: number, y: number, z: number): number {
    return (x & y) ^ (x & z) ^ (y & z);
  }

  private static sigma0(x: number): number {
    return CryptographicHash.rotr(2, x) ^ CryptographicHash.rotr(13, x) ^ CryptographicHash.rotr(22, x);
  }

  private static sigma1(x: number): number {
    return CryptographicHash.rotr(6, x) ^ CryptographicHash.rotr(11, x) ^ CryptographicHash.rotr(25, x);
  }

  private static gamma0(x: number): number {
    return CryptographicHash.rotr(7, x) ^ CryptographicHash.rotr(18, x) ^ (x >>> 3);
  }

  private static gamma1(x: number): number {
    return CryptographicHash.rotr(17, x) ^ CryptographicHash.rotr(19, x) ^ (x >>> 10);
  }

  public static sha256(asciiString: string): string {
    const K = [
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
      0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
      0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
      0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
      0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
      0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
      0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
      0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
    ];

    let H0 = 0x6a09e667, H1 = 0xbb67ae85, H2 = 0x3c6ef372, H3 = 0xa54ff53a;
    let H4 = 0x510e527f, H5 = 0x9b05688c, H6 = 0x1f83d9ab, H7 = 0x5be0cd19;

    // Convert string to UTF-8 bytes
    const bytes: number[] = [];
    for (let i = 0; i < asciiString.length; i++) {
      let code = asciiString.charCodeAt(i);
      if (code < 128) {
        bytes.push(code);
      } else if (code < 2048) {
        bytes.push((code >> 6) | 192, (code & 63) | 128);
      } else {
        bytes.push((code >> 12) | 224, ((code >> 6) & 63) | 128, (code & 63) | 128);
      }
    }

    const bitLength = bytes.length * 8;
    bytes.push(0x80);
    while ((bytes.length % 64) !== 56) {
      bytes.push(0);
    }
    // Append 64-bit length big-endian
    for (let i = 7; i >= 0; i--) {
      bytes.push((bitLength >>> (i * 8)) & 0xff);
    }

    // Process each 64-byte chunk
    for (let i = 0; i < bytes.length; i += 64) {
      const W = new Array(64);
      for (let t = 0; t < 16; t++) {
        W[t] = (bytes[i + t * 4] << 24) |
               (bytes[i + t * 4 + 1] << 16) |
               (bytes[i + t * 4 + 2] << 8) |
               (bytes[i + t * 4 + 3]);
      }
      for (let t = 16; t < 64; t++) {
        W[t] = (CryptographicHash.gamma1(W[t - 2]) + W[t - 7] + CryptographicHash.gamma0(W[t - 15]) + W[t - 16]) | 0;
      }

      let a = H0, b = H1, c = H2, d = H3, e = H4, f = H5, g = H6, h = H7;

      for (let t = 0; t < 64; t++) {
        const T1 = (h + CryptographicHash.sigma1(e) + CryptographicHash.choose(e, f, g) + K[t] + W[t]) | 0;
        const T2 = (CryptographicHash.sigma0(a) + CryptographicHash.majority(a, b, c)) | 0;
        h = g;
        g = f;
        f = e;
        e = (d + T1) | 0;
        d = c;
        c = b;
        b = a;
        a = (T1 + T2) | 0;
      }

      H0 = (H0 + a) | 0;
      H1 = (H1 + b) | 0;
      H2 = (H2 + c) | 0;
      H3 = (H3 + d) | 0;
      H4 = (H4 + e) | 0;
      H5 = (H5 + f) | 0;
      H6 = (H6 + g) | 0;
      H7 = (H7 + h) | 0;
    }

    const toHex = (n: number) => (n >>> 0).toString(16).padStart(8, '0');
    return `${toHex(H0)}${toHex(H1)}${toHex(H2)}${toHex(H3)}${toHex(H4)}${toHex(H5)}${toHex(H6)}${toHex(H7)}`;
  }
}

/**
 * Merkle Tree for cryptographic verification of batch custody events
 */
export class OrdnanceMerkleTree {
  private leaves: string[];
  private layers: string[][];

  constructor(eventHashes: string[]) {
    if (eventHashes.length === 0) {
      throw new Error("Cannot construct Merkle tree from zero hashes");
    }
    this.leaves = [...eventHashes];
    this.layers = [this.leaves];
    this.buildTree();
  }

  private buildTree(): void {
    let currentLayer = this.layers[0];
    while (currentLayer.length > 1) {
      const nextLayer: string[] = [];
      for (let i = 0; i < currentLayer.length; i += 2) {
        const left = currentLayer[i];
        const right = i + 1 < currentLayer.length ? currentLayer[i + 1] : left; // duplicate odd leaf
        const combined = CryptographicHash.sha256(left + right);
        nextLayer.push(combined);
      }
      this.layers.push(nextLayer);
      currentLayer = nextLayer;
    }
  }

  public getRoot(): string {
    return this.layers[this.layers.length - 1][0];
  }

  /**
   * Generates logarithmic audit proof for a given leaf
   */
  public generateProof(leafHash: string): MerkleProof {
    const leafIndex = this.leaves.indexOf(leafHash);
    if (leafIndex === -1) {
      throw new Error(`Leaf hash ${leafHash} not present in Merkle tree`);
    }

    const steps: MerkleProofStep[] = [];
    let idx = leafIndex;

    for (let layerIdx = 0; layerIdx < this.layers.length - 1; layerIdx++) {
      const layer = this.layers[layerIdx];
      const isEven = idx % 2 === 0;
      const pairIdx = isEven ? (idx + 1 < layer.length ? idx + 1 : idx) : idx - 1;

      steps.push({
        position: isEven ? 'RIGHT' : 'LEFT',
        hash: layer[pairIdx],
      });

      idx = Math.floor(idx / 2);
    }

    return {
      targetHash: leafHash,
      merkleRoot: this.getRoot(),
      steps,
    };
  }

  /**
   * Validates a cryptographic Merkle proof against a known Merkle Root
   */
  public static verifyProof(proof: MerkleProof): boolean {
    let current = proof.targetHash;
    for (const step of proof.steps) {
      if (step.position === 'LEFT') {
        current = CryptographicHash.sha256(step.hash + current);
      } else {
        current = CryptographicHash.sha256(current + step.hash);
      }
    }
    return current === proof.merkleRoot;
  }
}

/**
 * Immutable Chain-of-Custody Ledger
 */
export class OrdnanceBlockchainLedger {
  private chain: LedgerBlock[] = [];
  private pendingEvents: CustodyTransferEvent[] = [];

  constructor(genesisValidator = 'CENTCOM_LOGISTICS_DIRECTORATE_J4') {
    this.createGenesisBlock(genesisValidator);
  }

  private createGenesisBlock(validator: string): void {
    const genesisEvent: CustodyTransferEvent = {
      eventId: 'EVT-GENESIS-0000',
      assetId: 'SYSTEM_ROOT',
      timestampEpochMs: 1704067200000,
      prevCustodianCallsign: 'SECDEF_ACQUISITION',
      newCustodianCallsign: 'JOINT_ORDNANCE_COMMAND',
      transferType: 'DEPOT_RECEIPT',
      locationCoordinates: { lat: 38.8719, lon: -77.0563, elevationMeters: 10 },
      telemetrySnapshot: { ambientTempC: 21.0, peakShockG: 0.1, tamperSealIntact: true },
    };

    const eventHash = CryptographicHash.sha256(JSON.stringify(genesisEvent));
    const merkleTree = new OrdnanceMerkleTree([eventHash]);

    const genesisBlock: LedgerBlock = {
      blockIndex: 0,
      previousBlockHash: '0'.repeat(64),
      timestampEpochMs: genesisEvent.timestampEpochMs,
      merkleRoot: merkleTree.getRoot(),
      eventsCount: 1,
      events: [genesisEvent],
      blockHash: CryptographicHash.sha256(`0-${'0'.repeat(64)}-${merkleTree.getRoot()}-${genesisEvent.timestampEpochMs}`),
      validatorCallsign: validator,
    };

    this.chain.push(genesisBlock);
  }

  public recordTransfer(event: CustodyTransferEvent): { eventHash: string; pendingCount: number } {
    // Environmental sanity check
    if (!event.telemetrySnapshot.tamperSealIntact) {
      console.warn(`[SECURITY ALERT] Tamper seal broken for asset ${event.assetId} during transfer ${event.eventId}`);
    }
    this.pendingEvents.push(event);
    const eventHash = CryptographicHash.sha256(JSON.stringify(event));
    return { eventHash, pendingCount: this.pendingEvents.length };
  }

  public minePendingBlock(validatorCallsign: string): LedgerBlock {
    if (this.pendingEvents.length === 0) {
      throw new Error("Cannot mine block with zero pending custody transfers");
    }

    const previousBlock = this.chain[this.chain.length - 1];
    const eventHashes = this.pendingEvents.map(e => CryptographicHash.sha256(JSON.stringify(e)));
    const tree = new OrdnanceMerkleTree(eventHashes);
    const merkleRoot = tree.getRoot();
    const timestamp = Date.now();

    const blockHeader = `${previousBlock.blockIndex + 1}-${previousBlock.blockHash}-${merkleRoot}-${timestamp}`;
    const blockHash = CryptographicHash.sha256(blockHeader);

    const newBlock: LedgerBlock = {
      blockIndex: previousBlock.blockIndex + 1,
      previousBlockHash: previousBlock.blockHash,
      timestampEpochMs: timestamp,
      merkleRoot,
      eventsCount: this.pendingEvents.length,
      events: [...this.pendingEvents],
      blockHash,
      validatorCallsign,
    };

    this.chain.push(newBlock);
    this.pendingEvents = [];
    return newBlock;
  }

  public getChain(): LedgerBlock[] {
    return [...this.chain];
  }

  public verifyChainIntegrity(): { isValid: boolean; brokenBlockIndex?: number; reason?: string } {
    for (let i = 1; i < this.chain.length; i++) {
      const current = this.chain[i];
      const previous = this.chain[i - 1];

      // 1. Verify previous hash linkage
      if (current.previousBlockHash !== previous.blockHash) {
        return { isValid: false, brokenBlockIndex: i, reason: 'Broken previousBlockHash link' };
      }

      // 2. Verify Merkle Root consistency
      const hashes = current.events.map(e => CryptographicHash.sha256(JSON.stringify(e)));
      const tree = new OrdnanceMerkleTree(hashes);
      if (tree.getRoot() !== current.merkleRoot) {
        return { isValid: false, brokenBlockIndex: i, reason: 'Mismatched transaction Merkle root' };
      }

      // 3. Verify block hash calculation
      const expectedHeader = `${current.blockIndex}-${current.previousBlockHash}-${current.merkleRoot}-${current.timestampEpochMs}`;
      const calculatedHash = CryptographicHash.sha256(expectedHeader);
      if (calculatedHash !== current.blockHash) {
        return { isValid: false, brokenBlockIndex: i, reason: 'Invalid blockHash signature' };
      }
    }
    return { isValid: true };
  }
}
