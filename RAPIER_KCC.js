/*
  Rapier Kinematic Character Controller Reference
  ----------------------------------------------------------------
  Goal:
  - Provide a reusable implementation skeleton for stable KCC behavior:
    moving platforms, rotating push, support probe, step/slope stability.

  Requirements:
  - THREE
  - RAPIER world instance
  - Game objects that expose:
    - uuid
    - enabled
    - physicsType ("CHARACTER", "DYNAMIC", etc)
    - body (Rapier rigid body)
    - collider (optional, if not present we use body.collider(0))
    - mesh (THREE Object3D, for world transforms and yaw)
    - physicsSize { x, y, z } for support probe sizing
*/

import * as THREE from "three";

const EPS = 0.000001;

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function vecLength2D(x, z) {
  return Math.hypot(Number(x) || 0, Number(z) || 0);
}

function ensureVector3(v) {
  return v || new THREE.Vector3();
}

function ensureQuaternion(q) {
  return q || new THREE.Quaternion();
}

function getBodyCollider(go) {
  if (!go || !go.body) return null;
  if (go.collider) return go.collider;
  if (typeof go.body.collider === "function" && typeof go.body.numColliders === "function" && go.body.numColliders() > 0) {
    return go.body.collider(0);
  }
  return null;
}

function getColliderHandleValue(collider) {
  if (!collider) return null;
  try {
    if (typeof collider.handle === "function") return collider.handle();
    if (collider.handle !== undefined) return collider.handle;
  } catch (_error) {}
  return null;
}

function getColliderHandleKey(collider) {
  const handle = getColliderHandleValue(collider);
  if (handle === null || handle === undefined) return null;
  return String(handle);
}

function computeBodyBounds(go, targetBox) {
  if (!go || !targetBox) return false;
  if (go.mesh) {
    targetBox.setFromObject(go.mesh);
    return !targetBox.isEmpty();
  }
  return false;
}

export class RapierKccPlugAndPlay {
  constructor({ physicsWorld, getGameObjects }) {
    this.physicsWorld = physicsWorld;
    this.getGameObjects = getGameObjects;
    this.characterController = null;

    this.tmp = {
      supportCharacterBounds: new THREE.Box3(),
      supportSurfaceBounds: new THREE.Box3(),
      kccPenCharacterBounds: new THREE.Box3(),
      kccPenSourceBounds: new THREE.Box3(),
      pushTempSourceDelta: new THREE.Vector3(),
      pushTempRelPrev: new THREE.Vector3(),
      pushTempRelRotated: new THREE.Vector3(),
      pushTempExpectedPoint: new THREE.Vector3(),
      supportSamplePattern: [
        [0, 0],
        [1, 0], [-1, 0], [0, 1], [0, -1],
        [0.7071, 0.7071], [0.7071, -0.7071], [-0.7071, 0.7071], [-0.7071, -0.7071]
      ]
    };

    this.time = { deltaTime: 1 / 60 };
    this.cfg = {
      defaultStepHeight: 0.35,
      defaultStepWidth: 0.08,
      defaultSnapDistance: 0.2,
      defaultMaxSlopeDeg: 60,
      gravityFallback: 15.0,
      minGravity: 0.001,
      maxFallSpeed: 60,
      groundedBias: -0.00005,
      groundedGraceTime: 0.08,
      verticalDeadzone: 0.0025
    };
  }

  initializeController() {
    if (!this.physicsWorld) return;
    this.characterController = this.physicsWorld.createCharacterController(0.02);
    if (!this.characterController) return;
    if (typeof this.characterController.setUp === "function") this.characterController.setUp({ x: 0, y: 1, z: 0 });
    if (typeof this.characterController.setNormalNudgeFactor === "function") this.characterController.setNormalNudgeFactor(0.02);
    if (typeof this.characterController.setSlideEnabled === "function") this.characterController.setSlideEnabled(true);
    if (typeof this.characterController.enableAutostep === "function") {
      this.characterController.enableAutostep(this.cfg.defaultStepHeight, this.cfg.defaultStepWidth, true);
    }
    if (typeof this.characterController.enableSnapToGround === "function") {
      this.characterController.enableSnapToGround(this.cfg.defaultSnapDistance);
    }
    if (typeof this.characterController.setApplyImpulsesToDynamicBodies === "function") {
      this.characterController.setApplyImpulsesToDynamicBodies(false);
    }
  }

  initCharacterRuntime(go) {
    if (!go || go.physicsType !== "CHARACTER") return;
    go._kccVerticalVelocity = 0;
    go._kccGrounded = false;
    go._kccJumpQueued = false;
    go._kccJumpCount = 0;
    go._kccGroundGraceTime = 0;
    go._kccDesiredVelocity = ensureVector3(go._kccDesiredVelocity).set(0, 0, 0);
    go._kccExternalDisplacement = ensureVector3(go._kccExternalDisplacement).set(0, 0, 0);
    go._kccForcedDisplacement = ensureVector3(go._kccForcedDisplacement).set(0, 0, 0);
    go._kccLastCorrectedMove = ensureVector3(go._kccLastCorrectedMove).set(0, 0, 0);
    go._kccPlatformDelta = ensureVector3(go._kccPlatformDelta).set(0, 0, 0);
    go._kccPlatformDeltaQuat = ensureQuaternion(go._kccPlatformDeltaQuat).identity();
  }

  beforePhysicsStep(deltaTime) {
    this.time.deltaTime = deltaTime;
    const gameObjects = this.getGameObjects();
    for (const go of gameObjects) {
      if (!go || go.enabled === false) continue;
      if (!go._contactMotionPrevPos) go._contactMotionPrevPos = new THREE.Vector3();
      if (!go._contactMotionCurrPos) go._contactMotionCurrPos = new THREE.Vector3();
      if (!go._contactMotionPrevQuat) go._contactMotionPrevQuat = new THREE.Quaternion();
      if (!go._contactMotionCurrQuat) go._contactMotionCurrQuat = new THREE.Quaternion();
      if (!go._contactMotionDeltaQuat) go._contactMotionDeltaQuat = new THREE.Quaternion();

      if (go.body && typeof go.body.translation === "function") {
        const t = go.body.translation();
        go._contactMotionPrevPos.copy(go._contactMotionCurrPos);
        go._contactMotionCurrPos.set(Number(t.x) || 0, Number(t.y) || 0, Number(t.z) || 0);
      }
      if (go.body && typeof go.body.rotation === "function") {
        const r = go.body.rotation();
        go._contactMotionPrevQuat.copy(go._contactMotionCurrQuat);
        go._contactMotionCurrQuat.set(Number(r.x) || 0, Number(r.y) || 0, Number(r.z) || 0, Number(r.w) || 1);
        go._contactMotionDeltaQuat.copy(go._contactMotionCurrQuat).multiply(go._contactMotionPrevQuat.clone().invert());
      }
    }
  }

  isFullyLockedDynamic(obj) {
    if (!obj || String(obj.physicsType || "").toUpperCase() !== "DYNAMIC") return false;
    const lf = obj.linearFactor;
    const af = obj.angularFactor;
    const linearLocked = lf && Math.abs(Number(lf.x) || 0) <= EPS && Math.abs(Number(lf.y) || 0) <= EPS && Math.abs(Number(lf.z) || 0) <= EPS;
    const angularLocked = af && Math.abs(Number(af.x) || 0) <= EPS && Math.abs(Number(af.y) || 0) <= EPS && Math.abs(Number(af.z) || 0) <= EPS;
    return !!(linearLocked && angularLocked);
  }

  getGameObjectFromBody(body) {
    if (!body) return null;
    const gameObjects = this.getGameObjects();
    const bodyUuid = body.userData && body.userData.uuid ? String(body.userData.uuid) : "";
    if (bodyUuid) {
      const byUuid = gameObjects.find((go) => go && go.uuid === bodyUuid);
      if (byUuid) return byUuid;
    }
    return gameObjects.find((go) => go && go.body === body) || null;
  }

  updatePlatformCarryState() {
    const gameObjects = this.getGameObjects();
    for (const go of gameObjects) {
      if (!go || go.enabled === false) continue;
      go._hadPlatformContact = !!go._hasPlatformContact;
      go._hadKccGrounded = !!go._kccGrounded;

      if (go.physicsType !== "CHARACTER") {
        go._platformLastPos = null;
        go._platformLastQuat = null;
        go._platformLastSupportId = null;
        go._standingOn = null;
        go._hasPlatformContact = false;
        continue;
      }

      go._kccPlatformDelta = ensureVector3(go._kccPlatformDelta).set(0, 0, 0);
      go._kccPlatformDeltaQuat = ensureQuaternion(go._kccPlatformDeltaQuat).identity();

      if (go._hasPlatformContact && go._standingOn && go._standingOn.mesh && go._kccGrounded) {
        const platform = go._standingOn;
        const platformId = platform.uuid ? String(platform.uuid) : "";
        const platformPos = new THREE.Vector3();
        const platformQuat = new THREE.Quaternion();
        platform.mesh.getWorldPosition(platformPos);
        platform.mesh.getWorldQuaternion(platformQuat);

        const lastSupportId = go._platformLastSupportId ? String(go._platformLastSupportId) : "";
        const supportChanged = !!(platformId && lastSupportId && platformId !== lastSupportId);
        if (supportChanged) {
          go._platformLastPos = platformPos.clone();
          go._platformLastQuat = platformQuat.clone();
          go._platformLastSupportId = platformId;
        } else {
          if (go._platformLastPos && go._platformLastQuat && go.body) {
            const rbPos = go.body.translation();
            const charPos = new THREE.Vector3(rbPos.x, rbPos.y, rbPos.z);
            const lastQuatInv = go._platformLastQuat.clone().invert();
            const deltaQuat = platformQuat.clone().multiply(lastQuatInv);
            const lastOffset = charPos.sub(go._platformLastPos);
            const rotatedOffset = lastOffset.clone().applyQuaternion(deltaQuat);
            const pureTranslation = platformPos.clone().sub(go._platformLastPos);
            const rotationTranslation = rotatedOffset.sub(lastOffset);
            const combinedDelta = pureTranslation.add(rotationTranslation);
            combinedDelta.y = clamp(Number(combinedDelta.y) || 0, -0.25, 0.25);
            const carryLen = vecLength2D(combinedDelta.x, combinedDelta.z);
            if (carryLen > 0.40 && carryLen > 1e-8) {
              const s = 0.40 / carryLen;
              combinedDelta.x *= s;
              combinedDelta.z *= s;
            }
            go._kccPlatformDelta.copy(combinedDelta);

            const dEuler = new THREE.Euler().setFromQuaternion(deltaQuat, "YXZ");
            const yawOnly = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, dEuler.y, 0, "YXZ"));
            go._kccPlatformDeltaQuat.copy(yawOnly);
          }
          go._platformLastPos = platformPos.clone();
          go._platformLastQuat = platformQuat.clone();
          go._platformLastSupportId = platformId;
        }
      } else {
        go._platformLastPos = null;
        go._platformLastQuat = null;
        go._platformLastSupportId = null;
        go._standingOn = null;
      }

      go._hasPlatformContact = false;
    }
  }

  resolveSupportByProbe(characterGo) {
    if (!characterGo || characterGo.enabled === false || String(characterGo.physicsType || "").toUpperCase() !== "CHARACTER") return null;
    if (!characterGo.body || typeof characterGo.body.translation !== "function" || typeof RAPIER === "undefined") return null;
    if (!this.physicsWorld || typeof this.physicsWorld.castRay !== "function") return null;

    const translation = characterGo.body.translation();
    if (!translation) return null;

    const px = Number(translation.x) || 0;
    const py = Number(translation.y) || 0;
    const pz = Number(translation.z) || 0;
    const sizeX = Math.max(0.05, Number(characterGo.physicsSize && characterGo.physicsSize.x) || 0.5);
    const sizeY = Math.max(0.10, Number(characterGo.physicsSize && characterGo.physicsSize.y) || 1.0);
    const sizeZ = Math.max(0.05, Number(characterGo.physicsSize && characterGo.physicsSize.z) || 0.5);
    const halfHeight = sizeY * 0.5;
    const baseRadius = Math.max(0.04, Math.min(sizeX, sizeZ) * 0.5);
    const probeRadius = Math.max(0.03, baseRadius * 0.72);
    const probeLift = Math.max(0.03, Math.min(0.20, baseRadius * 0.90));
    const stepHeight = Math.max(0, Number(characterGo.characterStepHeight ?? characterGo.stepHeight ?? this.cfg.defaultStepHeight) || this.cfg.defaultStepHeight);
    const snapDistance = Math.max(0, Number(characterGo.characterSnapToGround ?? this.cfg.defaultSnapDistance) || this.cfg.defaultSnapDistance);
    const rayDistance = Math.max(0.14, stepHeight + snapDistance + 0.10);
    const startY = py - halfHeight + probeLift;
    const downDir = { x: 0, y: -1, z: 0 };

    let bestSurface = null;
    let bestToi = Infinity;
    for (const sample of this.tmp.supportSamplePattern) {
      const origin = { x: px + (sample[0] * probeRadius), y: startY, z: pz + (sample[1] * probeRadius) };
      const ray = new RAPIER.Ray(origin, downDir);
      let hit = null;
      if (typeof this.physicsWorld.castRayAndGetNormal === "function") {
        try { hit = this.physicsWorld.castRayAndGetNormal(ray, rayDistance, true, null, null, null, characterGo.body); } catch (_error) {}
      }
      if (!hit) {
        try { hit = this.physicsWorld.castRay(ray, rayDistance, true, null, null, null, characterGo.body); } catch (_error) {}
      }
      if (!hit || !hit.collider) continue;
      const hitToiRaw = Number(hit.toi);
      const hitToiAlt = Number(hit.timeOfImpact);
      const hitToi = Number.isFinite(hitToiRaw) ? hitToiRaw : hitToiAlt;
      if (!Number.isFinite(hitToi) || hitToi < 0 || hitToi > (rayDistance + 0.0001)) continue;
      const hitNormalY = Number(hit.normal && hit.normal.y);
      if (Number.isFinite(hitNormalY) && hitNormalY < 0.20) continue;
      const hitCollider = hit.collider;
      if (hitCollider && typeof hitCollider.isSensor === "function" && hitCollider.isSensor()) continue;
      const hitBody = hitCollider && typeof hitCollider.parent === "function" ? hitCollider.parent() : null;
      const surfaceGo = this.getGameObjectFromBody(hitBody);
      if (!surfaceGo || surfaceGo === characterGo || surfaceGo.enabled === false) continue;
      const surfaceType = String(surfaceGo.physicsType || "").toUpperCase();
      if (surfaceType === "NONE" || surfaceType === "TRIGGER" || surfaceType === "CHARACTER") continue;
      if (hitToi < bestToi) {
        bestToi = hitToi;
        bestSurface = surfaceGo;
      }
    }
    return bestSurface;
  }

  isCharacterSupportedBySurface(characterGo, surfaceGo) {
    if (!characterGo || !surfaceGo || characterGo === surfaceGo) return false;
    if (String(characterGo.physicsType || "").toUpperCase() !== "CHARACTER") return false;
    const surfaceType = String(surfaceGo.physicsType || "").toUpperCase();
    if (surfaceType === "NONE" || surfaceType === "TRIGGER" || surfaceType === "CHARACTER") return false;
    if (!computeBodyBounds(characterGo, this.tmp.supportCharacterBounds)) return false;
    if (!computeBodyBounds(surfaceGo, this.tmp.supportSurfaceBounds)) return false;

    const a = this.tmp.supportCharacterBounds;
    const b = this.tmp.supportSurfaceBounds;
    const overlapX = Math.min(a.max.x, b.max.x) - Math.max(a.min.x, b.min.x);
    const overlapZ = Math.min(a.max.z, b.max.z) - Math.max(a.min.z, b.min.z);
    if (overlapX <= 0.001 || overlapZ <= 0.001) return false;
    const characterBottom = a.min.y;
    const characterCenterY = (a.min.y + a.max.y) * 0.5;
    const surfaceTop = b.max.y;
    const surfaceCenterY = (b.min.y + b.max.y) * 0.5;
    const verticalGap = characterBottom - surfaceTop;
    const supportTolerance = 0.10;
    if (verticalGap < -supportTolerance) return false;
    if (verticalGap > supportTolerance) return false;
    if (characterCenterY <= surfaceCenterY) return false;
    return true;
  }

  accumulateCharacterExternalPush(characterGo, sourceGo, isSupportContact = false) {
    if (!characterGo || !sourceGo || characterGo === sourceGo) return;
    if (String(characterGo.physicsType || "").toUpperCase() !== "CHARACTER") return;
    const sourceType = String(sourceGo.physicsType || "").toUpperCase();
    if (sourceType !== "DYNAMIC" && sourceType !== "CHARACTER") return;
    if (!characterGo.body || !sourceGo.body || isSupportContact) return;

    characterGo._kccExternalDisplacement = ensureVector3(characterGo._kccExternalDisplacement);
    characterGo._kccForcedDisplacement = ensureVector3(characterGo._kccForcedDisplacement);
    const pushAccumulator = characterGo._kccExternalDisplacement;
    const forcedAccumulator = characterGo._kccForcedDisplacement;
    const isFullyLockedDynamicSource = this.isFullyLockedDynamic(sourceGo);
    const dtRaw = (Number.isFinite(this.time.deltaTime) && this.time.deltaTime > 0) ? this.time.deltaTime : (1 / 60);
    const dt = Math.min(Math.max(dtRaw, 1 / 240), 1 / 20);

    const push = this.tmp.pushTempSourceDelta.set(0, 0, 0);
    if (sourceType === "DYNAMIC" && typeof sourceGo.body.linvel === "function") {
      const charPos = characterGo.body.translation();
      let usedTransformDelta = false;
      if (charPos && sourceGo._contactMotionPrevPos && sourceGo._contactMotionCurrPos && sourceGo._contactMotionDeltaQuat) {
        this.tmp.pushTempRelPrev.set(
          (Number(charPos.x) || 0) - sourceGo._contactMotionPrevPos.x,
          (Number(charPos.y) || 0) - sourceGo._contactMotionPrevPos.y,
          (Number(charPos.z) || 0) - sourceGo._contactMotionPrevPos.z
        );
        this.tmp.pushTempRelRotated.copy(this.tmp.pushTempRelPrev).applyQuaternion(sourceGo._contactMotionDeltaQuat);
        this.tmp.pushTempExpectedPoint.copy(sourceGo._contactMotionCurrPos).add(this.tmp.pushTempRelRotated);
        push.set(
          this.tmp.pushTempExpectedPoint.x - (Number(charPos.x) || 0),
          0,
          this.tmp.pushTempExpectedPoint.z - (Number(charPos.z) || 0)
        );
        usedTransformDelta = push.lengthSq() > 1e-10;
      }
      if (!usedTransformDelta && typeof sourceGo.body.velocityAtPoint === "function" && charPos) {
        const velAtPoint = sourceGo.body.velocityAtPoint({
          x: Number(charPos.x) || 0,
          y: Number(charPos.y) || 0,
          z: Number(charPos.z) || 0
        });
        push.set((Number(velAtPoint.x) || 0) * dt, 0, (Number(velAtPoint.z) || 0) * dt);
      } else if (!usedTransformDelta) {
        const lv = sourceGo.body.linvel();
        push.set((Number(lv.x) || 0) * dt, 0, (Number(lv.z) || 0) * dt);
      }
    } else if (sourceType === "CHARACTER" && sourceGo._kccLastCorrectedMove) {
      push.copy(sourceGo._kccLastCorrectedMove);
      push.y = 0;
    }

    if (push.lengthSq() <= 1e-8) return;
    const sourceLen = vecLength2D(push.x, push.z);
    if (sourceLen <= 1e-8) return;
    const pushScale = sourceType === "DYNAMIC" ? 1.0 : 0.75;
    const pushMax = isFullyLockedDynamicSource ? 0.95 : 0.35;
    const applied = Math.min(pushMax, sourceLen * pushScale);
    const nx = push.x / sourceLen;
    const nz = push.z / sourceLen;
    pushAccumulator.x += nx * applied;
    pushAccumulator.z += nz * applied;
    if (isFullyLockedDynamicSource) {
      forcedAccumulator.x += nx * applied;
      forcedAccumulator.z += nz * applied;
    }

    const maxAccum = isFullyLockedDynamicSource ? 1.20 : 0.45;
    const accumLen = vecLength2D(pushAccumulator.x, pushAccumulator.z);
    if (accumLen > maxAccum && accumLen > 1e-8) {
      const s = maxAccum / accumLen;
      pushAccumulator.x *= s;
      pushAccumulator.z *= s;
    }
    if (isFullyLockedDynamicSource) {
      const forcedLen = vecLength2D(forcedAccumulator.x, forcedAccumulator.z);
      if (forcedLen > maxAccum && forcedLen > 1e-8) {
        const s = maxAccum / forcedLen;
        forcedAccumulator.x *= s;
        forcedAccumulator.z *= s;
      }
    }
  }

  collectCollisionPushes() {
    if (!this.physicsWorld || !this.physicsWorld.narrowPhase) return;
    const colliderByHandle = new Map();
    this.physicsWorld.forEachCollider((collider) => {
      const handleKey = getColliderHandleKey(collider);
      if (handleKey !== null) colliderByHandle.set(handleKey, collider);
    });
    const resolveColliderFromArg = (arg) => {
      if (arg && typeof arg === "object") return arg;
      if (arg === null || arg === undefined) return null;
      const key = String(arg);
      if (colliderByHandle.has(key)) return colliderByHandle.get(key) || null;
      if (typeof this.physicsWorld.getCollider === "function") {
        try {
          const found = this.physicsWorld.getCollider(arg);
          if (found) return found;
        } catch (_error) {}
      }
      return null;
    };

    const hasNarrowPhaseTouch = (a, b) => {
      let touching = false;
      let attempted = false;
      const testPair = (pa, pb) => {
        if (touching) return;
        try {
          attempted = true;
          this.physicsWorld.narrowPhase.contactPair(pa, pb, (manifold) => {
            const n = (typeof manifold.numContacts === "function") ? (Number(manifold.numContacts()) || 0) : 0;
            if (n > 0) {
              for (let i = 0; i < n; i++) {
                const dist = (typeof manifold.contactDist === "function") ? Number(manifold.contactDist(i)) : 0;
                if (!Number.isFinite(dist) || dist <= 0.04) {
                  touching = true;
                  return;
                }
              }
            }
            if (!touching && typeof manifold.numSolverContacts === "function" && (Number(manifold.numSolverContacts()) || 0) > 0) {
              touching = true;
            }
          });
        } catch (_error) {}
      };
      const ha = getColliderHandleValue(a);
      const hb = getColliderHandleValue(b);
      testPair(ha, hb);
      testPair(a, b);
      if (touching) return true;
      if (attempted) return false;
      return null;
    };

    const processedPairs = new Set();
    this.physicsWorld.forEachCollider((collider) => {
      const invokeContactPairs = (queryArg) => {
        if (queryArg === null || queryArg === undefined) return;
        try {
          this.physicsWorld.narrowPhase.contactPairsWith(queryArg, (otherArg) => {
            const otherCollider = resolveColliderFromArg(otherArg);
            if (!otherCollider) return;
            const h1 = getColliderHandleKey(collider);
            const h2 = getColliderHandleKey(otherCollider);
            if (h1 === null || h2 === null) return;
            const pairKey = h1 < h2 ? `${h1}-${h2}` : `${h2}-${h1}`;
            if (processedPairs.has(pairKey)) return;
            processedPairs.add(pairKey);

            const body1 = collider.parent();
            const body2 = otherCollider.parent();
            if (!body1 || !body2) return;
            const go1 = this.getGameObjectFromBody(body1);
            const go2 = this.getGameObjectFromBody(body2);
            if (!go1 || !go2 || go1.enabled === false || go2.enabled === false) return;
            if (go1.physicsType === "TRIGGER" || go2.physicsType === "TRIGGER") return;

            const allowPush = hasNarrowPhaseTouch(collider, otherCollider) === true;
            if (!allowPush) return;

            const go1SupportedByGo2 = go1.physicsType === "CHARACTER" && this.isCharacterSupportedBySurface(go1, go2);
            const go2SupportedByGo1 = go2.physicsType === "CHARACTER" && this.isCharacterSupportedBySurface(go2, go1);
            if (go1.physicsType === "CHARACTER") this.accumulateCharacterExternalPush(go1, go2, go1SupportedByGo2);
            if (go2.physicsType === "CHARACTER") this.accumulateCharacterExternalPush(go2, go1, go2SupportedByGo1);
          });
        } catch (_error) {}
      };
      invokeContactPairs(collider);
      invokeContactPairs(getColliderHandleValue(collider));
    });
  }

  updateResolvedSupport() {
    const gameObjects = this.getGameObjects();
    for (const go of gameObjects) {
      if (!go || go.enabled === false || String(go.physicsType || "").toUpperCase() !== "CHARACTER") continue;
      const supportGo = this.resolveSupportByProbe(go);
      if (supportGo) {
        go._standingOn = supportGo;
        go._hasPlatformContact = true;
      } else {
        go._standingOn = null;
      }
    }
  }

  solveCharacters(deltaTime) {
    if (!this.characterController) return;
    const gameObjects = this.getGameObjects();
    for (const go of gameObjects) {
      if (!go || go.enabled === false || go.physicsType !== "CHARACTER" || !go.body) continue;
      const collider = getBodyCollider(go);
      if (!collider) continue;

      const desiredVelocity = go._kccDesiredVelocity || new THREE.Vector3();
      const desiredMove = new THREE.Vector3((Number(desiredVelocity.x) || 0) * deltaTime, 0, (Number(desiredVelocity.z) || 0) * deltaTime);
      if (!Number.isFinite(go._kccGroundGraceTime)) go._kccGroundGraceTime = 0;
      if (go._hadPlatformContact && go._hadKccGrounded && go._kccPlatformDelta) desiredMove.add(go._kccPlatformDelta);
      if (go._kccExternalDisplacement) desiredMove.add(go._kccExternalDisplacement);

      if (go._kccForcedDisplacement) {
        const fx = Number(go._kccForcedDisplacement.x) || 0;
        const fz = Number(go._kccForcedDisplacement.z) || 0;
        const fl = vecLength2D(fx, fz);
        if (fl > 1e-10) {
          const nx = fx / fl;
          const nz = fz / fl;
          const currentAlong = (desiredMove.x * nx) + (desiredMove.z * nz);
          if (currentAlong < fl) {
            const add = fl - currentAlong;
            desiredMove.x += nx * add;
            desiredMove.z += nz * add;
          }
        }
      }

      const maxJumps = Math.max(1, Number(go._kccMaxJumps) || 1);
      if (go._kccJumpQueued) {
        if (go._kccGrounded) go._kccJumpCount = 0;
        if (go._kccGrounded || go._kccJumpCount < maxJumps) {
          go._kccVerticalVelocity = Number(go._kccJumpSpeed) || 5;
          go._kccJumpCount += 1;
          go._kccGrounded = false;
        }
        go._kccJumpQueued = false;
      }

      const gravity = Math.max(this.cfg.minGravity, go.fallSpeed !== undefined ? go.fallSpeed : this.cfg.gravityFallback);
      if (go._kccGrounded && go._kccVerticalVelocity <= 0) {
        go._kccVerticalVelocity = 0;
        desiredMove.y += this.cfg.groundedBias;
      } else {
        go._kccVerticalVelocity -= gravity * deltaTime;
        if (go._kccVerticalVelocity < -this.cfg.maxFallSpeed) go._kccVerticalVelocity = -this.cfg.maxFallSpeed;
        desiredMove.y += go._kccVerticalVelocity * deltaTime;
      }

      if (typeof this.characterController.enableAutostep === "function") {
        const stepHeight = Math.max(0, Number(go.characterStepHeight ?? go.stepHeight ?? this.cfg.defaultStepHeight) || this.cfg.defaultStepHeight);
        const stepWidth = Math.max(0, Number(go.characterStepMinWidth ?? go.stepMinWidth ?? this.cfg.defaultStepWidth) || this.cfg.defaultStepWidth);
        this.characterController.enableAutostep(stepHeight, stepWidth, true);
      }
      if (typeof this.characterController.enableSnapToGround === "function") {
        const snapDistance = Math.max(0, Number(go.characterSnapToGround ?? this.cfg.defaultSnapDistance) || this.cfg.defaultSnapDistance);
        this.characterController.enableSnapToGround(snapDistance);
      }
      if (typeof this.characterController.setMaxSlopeClimbAngle === "function") {
        const maxSlopeDeg = Math.max(1, Math.min(89.5, Number(go.characterMaxSlopeAngleDeg ?? this.cfg.defaultMaxSlopeDeg) || this.cfg.defaultMaxSlopeDeg));
        const maxSlopeRad = (maxSlopeDeg * Math.PI) / 180;
        this.characterController.setMaxSlopeClimbAngle(maxSlopeRad);
        if (typeof this.characterController.setMinSlopeSlideAngle === "function") {
          const slideSlopeDeg = Math.max(maxSlopeDeg + 0.5, Math.min(89.9, maxSlopeDeg + 2));
          this.characterController.setMinSlopeSlideAngle((slideSlopeDeg * Math.PI) / 180);
        }
      }

      this.characterController.computeColliderMovement(collider, desiredMove);
      let corrected = this.characterController.computedMovement();

      let groundedNow = !!this.characterController.computedGrounded();
      if (groundedNow) {
        go._kccGroundGraceTime = this.cfg.groundedGraceTime;
      } else if ((go._kccGroundGraceTime > 0) && (Number(go._kccVerticalVelocity) <= 0.05) && !go._kccJumpQueued) {
        groundedNow = true;
        go._kccGroundGraceTime = Math.max(0, go._kccGroundGraceTime - deltaTime);
      } else {
        go._kccGroundGraceTime = 0;
      }

      let correctedX = Number(corrected.x) || 0;
      let correctedY = Number(corrected.y) || 0;
      let correctedZ = Number(corrected.z) || 0;
      if (groundedNow && Math.abs(correctedY) < this.cfg.verticalDeadzone) correctedY = 0;
      const currentPos = go.body.translation();
      const nextPos = { x: currentPos.x + correctedX, y: currentPos.y + correctedY, z: currentPos.z + correctedZ };
      if (typeof go.body.setNextKinematicTranslation === "function") go.body.setNextKinematicTranslation(nextPos);
      else go.body.setTranslation(nextPos, true);

      const currentMeshQuat = go.mesh ? go.mesh.quaternion.clone() : new THREE.Quaternion();
      const uprightEuler = new THREE.Euler().setFromQuaternion(currentMeshQuat, "YXZ");
      const targetQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, uprightEuler.y, 0, "YXZ"));
      if (go._hadPlatformContact && go._kccPlatformDeltaQuat) targetQuat.premultiply(go._kccPlatformDeltaQuat);
      if (typeof go.body.setNextKinematicRotation === "function") {
        go.body.setNextKinematicRotation({ x: targetQuat.x, y: targetQuat.y, z: targetQuat.z, w: targetQuat.w });
      } else {
        go.body.setRotation({ x: targetQuat.x, y: targetQuat.y, z: targetQuat.z, w: targetQuat.w }, true);
      }

      go._kccGrounded = groundedNow;
      go._kccLastCorrectedMove = ensureVector3(go._kccLastCorrectedMove).set(correctedX, correctedY, correctedZ);
      if (groundedNow) {
        if (go._kccVerticalVelocity < 0) go._kccVerticalVelocity = 0;
        go._kccJumpCount = 0;
      }

      if (go._kccDesiredVelocity) go._kccDesiredVelocity.set(0, 0, 0);
      if (go._kccExternalDisplacement) go._kccExternalDisplacement.set(0, 0, 0);
      if (go._kccForcedDisplacement) go._kccForcedDisplacement.set(0, 0, 0);
    }
  }

  /*
    Call order every frame:
    1) beforePhysicsStep(dt)
    2) updatePlatformCarryState()
    3) collectCollisionPushes()
    4) updateResolvedSupport()
    5) solveCharacters(dt)
  */
}

