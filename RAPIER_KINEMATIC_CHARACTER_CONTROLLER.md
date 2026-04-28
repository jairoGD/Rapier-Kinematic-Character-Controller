# Rapier Kinematic Character Controller (Reference Implementation)

This document is a practical reference for building a stable 3D kinematic character controller with Rapier.
It focuses on behavior that is usually missing in minimal examples:

- stable grounding
- autostep + slope limits
- moving platform carry (translation + rotation)
- rotating platform side push
- anti-penetration when player input fights moving obstacles

The snippets are written in JavaScript-style pseudocode and can be adapted to TypeScript/C#/Rust.

## 1) Design goals

1. Character always stays upright.
2. Input-driven movement feels deterministic.
3. Character can be carried by moving/rotating platforms.
4. Locked dynamic platforms can push the character reliably.
5. No random launch/flicker on steps, edges, or support transitions.

## 2) Runtime state (per character)

```js
const kccState = {
  desiredVelocity: vec3(0, 0, 0),   // set by gameplay input each frame
  verticalVelocity: 0,              // gravity + jump
  grounded: false,
  jumpQueued: false,
  jumpCount: 0,
  maxJumps: 1,
  jumpSpeed: 5.0,

  externalDisplacement: vec3(0, 0, 0), // pushes from contacts
  forcedDisplacement: vec3(0, 0, 0),   // cannot be canceled by opposite input
  lastCorrectedMove: vec3(0, 0, 0),

  // moving-platform carry
  standingOn: null,                  // current support object
  hadPlatformContact: false,         // previous frame
  hadGrounded: false,                // previous frame
  platformDelta: vec3(0, 0, 0),      // carry translation for next frame
  platformDeltaYawQuat: quatIdentity(),
  platformLastPos: null,
  platformLastQuat: null,
  platformLastSupportId: null,

  // stability
  groundGraceTime: 0.0
};
```

## 3) Tunables (good defaults)

```js
const cfg = {
  gravity: 15.0,
  maxFallSpeed: 60.0,

  stepHeight: 0.35,
  stepMinWidth: 0.08,
  snapDistance: 0.2,

  maxSlopeDeg: 60.0,
  // slide threshold should be slightly above climb threshold
  minSlideSlopeDeg: 62.0,

  groundedBias: -0.00005,     // tiny downward bias
  verticalDeadzone: 0.0025,   // remove tiny vertical jitter while grounded
  groundedGrace: 0.08         // hysteresis to avoid grounded flicker
};
```

## 4) Create and configure Rapier KCC

```js
const characterController = world.createCharacterController(0.02);
characterController.setUp({ x: 0, y: 1, z: 0 });
characterController.setNormalNudgeFactor(0.02);
characterController.setSlideEnabled(true);
characterController.enableAutostep(cfg.stepHeight, cfg.stepMinWidth, true);
characterController.enableSnapToGround(cfg.snapDistance);
characterController.setApplyImpulsesToDynamicBodies(false); // we inject push manually
```

## 5) Per-frame update pipeline

Call once per character after gameplay input has written `desiredVelocity`.

```js
function updateCharacterKCC(go, dt) {
  const st = go.kccState;
  const collider = getMainCollider(go);
  if (!collider) return;

  // 1) start from input velocity
  const desiredMove = vec3(
    st.desiredVelocity.x * dt,
    0,
    st.desiredVelocity.z * dt
  );

  // 2) add platform carry from previous support frame
  if (st.hadPlatformContact && st.hadGrounded) {
    desiredMove.add(st.platformDelta);
  }

  // 3) add world-space pushes accumulated from contacts
  desiredMove.add(st.externalDisplacement);

  // 4) enforce forced push component (cannot be canceled by opposite input)
  applyForcedPushLane(desiredMove, st.forcedDisplacement);

  // 5) jump + gravity
  integrateJumpAndGravity(go, desiredMove, dt);

  // 6) configure autostep/slope each frame (allows per-object tuning)
  configureKCCFromObject(go);

  // 7) solve pass 1
  characterController.computeColliderMovement(collider, desiredMove);
  let corrected = characterController.computedMovement();

  // 8) optional pass 2: compensation against locked moving/rotating obstacles
  corrected = applyCompensationPassIfNeeded(go, collider, desiredMove, corrected);

  // 9) grounded with hysteresis
  let groundedNow = !!characterController.computedGrounded();
  groundedNow = applyGroundedHysteresis(go, groundedNow, dt);

  // 10) kill tiny vertical jitter while grounded
  if (groundedNow && Math.abs(corrected.y) < cfg.verticalDeadzone) corrected.y = 0;

  // 11) apply corrected translation to kinematic body
  applyKinematicTranslation(go.body, corrected);

  // 12) keep upright + optionally inherit support yaw
  applyUprightRotationWithPlatformYaw(go, st.platformDeltaYawQuat, st.hadPlatformContact);

  // 13) finalize state
  st.grounded = groundedNow;
  st.lastCorrectedMove.copy(corrected);
  if (groundedNow && st.verticalVelocity < 0) st.verticalVelocity = 0;
  if (groundedNow) st.jumpCount = 0;

  // 14) consume one-frame intents to avoid stale sliding
  st.desiredVelocity.set(0, 0, 0);
  st.externalDisplacement.set(0, 0, 0);
  st.forcedDisplacement.set(0, 0, 0);
}
```

## 6) Jump and gravity integration

```js
function integrateJumpAndGravity(go, desiredMove, dt) {
  const st = go.kccState;
  const maxJumps = Math.max(1, st.maxJumps || 1);

  if (st.jumpQueued) {
    if (st.grounded) st.jumpCount = 0;
    if (st.grounded || st.jumpCount < maxJumps) {
      st.verticalVelocity = st.jumpSpeed || 5.0;
      st.jumpCount += 1;
      st.grounded = false;
    }
    st.jumpQueued = false;
  }

  if (st.grounded && st.verticalVelocity <= 0) {
    st.verticalVelocity = 0;
    desiredMove.y += cfg.groundedBias;
  } else {
    st.verticalVelocity -= cfg.gravity * dt;
    if (st.verticalVelocity < -cfg.maxFallSpeed) st.verticalVelocity = -cfg.maxFallSpeed;
    desiredMove.y += st.verticalVelocity * dt;
  }
}
```

## 7) Forced push lane (key for moving platform reliability)

Without this, opposite player input can cancel platform push and create penetration.

```js
function applyForcedPushLane(desiredMove, forced) {
  const fx = forced.x || 0;
  const fz = forced.z || 0;
  const lenSq = fx * fx + fz * fz;
  if (lenSq < 1e-10) return;

  const len = Math.sqrt(lenSq);
  const nx = fx / len;
  const nz = fz / len;
  const currentAlong = desiredMove.x * nx + desiredMove.z * nz;

  if (currentAlong < len) {
    const add = len - currentAlong;
    desiredMove.x += nx * add;
    desiredMove.z += nz * add;
  }
}
```

## 8) Support detection by geometry-aware probe

Do not rely only on center ray or AABB overlap. Use multiple samples around character base.

```js
function resolveSupportByProbe(character, world) {
  const t = character.body.translation();
  const size = character.physicsSize; // x,y,z

  const halfHeight = Math.max(0.1, size.y * 0.5);
  const baseRadius = Math.max(0.04, Math.min(size.x, size.z) * 0.5);
  const probeRadius = Math.max(0.03, baseRadius * 0.72);
  const probeLift = Math.max(0.03, Math.min(0.20, baseRadius * 0.90));
  const rayDistance = Math.max(0.14, cfg.stepHeight + cfg.snapDistance + 0.10);

  const startY = t.y - halfHeight + probeLift;
  const samples = [
    [0, 0], [1, 0], [-1, 0], [0, 1], [0, -1],
    [0.7071, 0.7071], [0.7071, -0.7071], [-0.7071, 0.7071], [-0.7071, -0.7071]
  ];

  let best = null;
  let bestToi = Infinity;

  for (const [sx, sz] of samples) {
    const origin = { x: t.x + sx * probeRadius, y: startY, z: t.z + sz * probeRadius };
    const ray = new RAPIER.Ray(origin, { x: 0, y: -1, z: 0 });
    const hit = world.castRayAndGetNormal(ray, rayDistance, true, null, null, null, character.body)
      || world.castRay(ray, rayDistance, true, null, null, null, character.body);
    if (!hit || !hit.collider) continue;

    const toi = Number.isFinite(hit.toi) ? hit.toi : hit.timeOfImpact;
    if (!Number.isFinite(toi) || toi < 0 || toi > rayDistance) continue;
    if (Number.isFinite(hit.normal?.y) && hit.normal.y < 0.20) continue;
    if (hit.collider.isSensor?.()) continue;

    const support = gameObjectFromRigidBody(hit.collider.parent?.());
    if (!support || support === character) continue;
    if (isUnsupportedPhysicsType(support)) continue;

    if (toi < bestToi) {
      bestToi = toi;
      best = support;
    }
  }

  return best;
}
```

## 9) Platform carry (translation + yaw)

At frame start, compute support transform delta from last frame, then store carry for character movement.

```js
function updatePlatformCarry(character) {
  const st = character.kccState;
  st.platformDelta.set(0, 0, 0);
  st.platformDeltaYawQuat.identity();

  if (!st.standingOn || !st.grounded) {
    st.platformLastPos = null;
    st.platformLastQuat = null;
    st.platformLastSupportId = null;
    return;
  }

  const support = st.standingOn;
  const pos = getWorldPosition(support);
  const quat = getWorldQuaternion(support);
  const supportId = support.uuid;

  // support changed: reset baseline to avoid one-frame launch
  if (st.platformLastSupportId && st.platformLastSupportId !== supportId) {
    st.platformLastPos = pos.clone();
    st.platformLastQuat = quat.clone();
    st.platformLastSupportId = supportId;
    return;
  }

  if (st.platformLastPos && st.platformLastQuat) {
    const dPos = pos.clone().sub(st.platformLastPos);
    const dQuat = quat.clone().multiply(st.platformLastQuat.clone().invert());

    // carry only world translation + yaw (character stays upright)
    st.platformDelta.copy(clampHorizontalDelta(dPos, 1.0));
    st.platformDeltaYawQuat.copy(extractYawQuat(dQuat));
  }

  st.platformLastPos = pos.clone();
  st.platformLastQuat = quat.clone();
  st.platformLastSupportId = supportId;
}
```

## 10) External push from moving bodies (including rotation)

Kinematic characters do not automatically receive impulses like dynamic bodies.
You must inject push displacement manually from contact pairs.

```js
function accumulateExternalPush(character, source, dt, isSupportContact) {
  if (isSupportContact) return; // support carry is handled separately

  const st = character.kccState;
  const push = vec3(0, 0, 0);

  // Preferred: displacement at character point caused by source transform delta.
  // This captures rotating platforms with non-circular geometry.
  const charPos = character.body.translation();
  if (source.prevPos && source.currPos && source.deltaQuat && charPos) {
    const relPrev = vec3(charPos.x - source.prevPos.x, charPos.y - source.prevPos.y, charPos.z - source.prevPos.z);
    const relRot = relPrev.clone().applyQuaternion(source.deltaQuat);
    const expected = source.currPos.clone().add(relRot);
    push.set(expected.x - charPos.x, 0, expected.z - charPos.z);
  } else if (source.body.velocityAtPoint && charPos) {
    const vp = source.body.velocityAtPoint(charPos);
    push.set(vp.x * dt, 0, vp.z * dt);
  } else {
    const lv = source.body.linvel();
    push.set(lv.x * dt, 0, lv.z * dt);
  }

  if (push.lengthSq() < 1e-8) return;

  // world-space push (never local-space)
  const isLockedDynamic = isFullyLockedDynamic(source);
  const scale = source.physicsType === "DYNAMIC" ? 1.0 : 0.75;
  const maxPush = isLockedDynamic ? 0.95 : 0.35;
  const applied = clampLengthXZ(push, maxPush).multiplyScalar(scale);
  st.externalDisplacement.add(applied);

  // for locked dynamic platforms, keep a forced lane
  if (isLockedDynamic) st.forcedDisplacement.add(applied);

  // global clamp
  clampLengthXZInPlace(st.externalDisplacement, isLockedDynamic ? 1.20 : 0.45);
  if (isLockedDynamic) clampLengthXZInPlace(st.forcedDisplacement, 1.20);
}
```

## 11) Compensation pass from Rapier computed collisions

After first solve, inspect collisions and add missing outward displacement against locked movers.

```js
function applyCompensationPassIfNeeded(go, collider, desiredMove, corrected) {
  let compensation = vec3(0, 0, 0);
  const n = characterController.numComputedCollisions?.() ?? 0;

  for (let i = 0; i < n; i++) {
    const col = characterController.computedCollision(i);
    const obstacle = gameObjectFromRigidBody(col?.collider?.parent?.());
    if (!isFullyLockedDynamic(obstacle)) continue;

    // Filter vestigial/pre-contact collisions
    const toi = Number(col?.toi?.() ?? col?.toi);
    if (Number.isFinite(toi) && toi > 0.20) continue;

    const w1 = col.witness1 ?? col.worldWitness1?.();
    const w2 = col.witness2 ?? col.worldWitness2?.();
    if (w1 && w2 && distance(w1, w2) > 0.06) continue;

    const n1 = col.normal1 ?? col.worldNormal1?.();
    if (!n1) continue;
    if ((n1.y ?? 0) > 0.50) continue; // ignore top/floor-like contacts

    const sideN = normalizeXZ(n1);
    if (!sideN) continue;

    const platformDeltaAtContact = computePlatformPointDelta(obstacle, w1);
    const platformOutward = dotXZ(platformDeltaAtContact, sideN);
    if (platformOutward <= 1e-6) continue;

    const correctedOutward = dotXZ(corrected, sideN);
    if (correctedOutward + 1e-6 < platformOutward) {
      const missing = Math.min(1.20, platformOutward - correctedOutward);
      compensation.x += sideN.x * missing;
      compensation.z += sideN.z * missing;
    }
  }

  if (compensation.lengthSq() < 1e-10) return corrected;

  clampLengthXZInPlace(compensation, 1.25);
  const secondDesired = desiredMove.clone().add(compensation);
  characterController.computeColliderMovement(collider, secondDesired);
  return characterController.computedMovement();
}
```

## 12) Keep upright + optional platform yaw carry

```js
function applyUprightRotationWithPlatformYaw(character, platformYawDelta, hadPlatformContact) {
  const meshQ = character.mesh.quaternion.clone();
  const yaw = extractYaw(meshQ);
  const target = quatFromYaw(yaw);

  if (hadPlatformContact) {
    // inherit yaw only, keep pitch/roll zero
    target.premultiply(platformYawDelta);
  }

  character.body.setNextKinematicRotation?.(toRapierQuat(target));
}
```

## 13) Grounded hysteresis

```js
function applyGroundedHysteresis(go, groundedNow, dt) {
  const st = go.kccState;
  if (groundedNow) {
    st.groundGraceTime = cfg.groundedGrace;
    return true;
  }
  if (st.groundGraceTime > 0 && st.verticalVelocity <= 0.05 && !st.jumpQueued) {
    st.groundGraceTime = Math.max(0, st.groundGraceTime - dt);
    return true;
  }
  st.groundGraceTime = 0;
  return false;
}
```

## 14) Character API contract (gameplay side)

Your gameplay layer should only write high-level intent:

```js
// every frame
character.kccState.desiredVelocity.set(vx, 0, vz);

// when jump requested
character.kccState.jumpQueued = true;
character.kccState.jumpSpeed = 5.0;
character.kccState.maxJumps = 1;
```

Do not write directly to body translation for normal movement; let KCC own locomotion.

## 15) Common failure modes and fixes

1. Character slides forever after key release:
   - consume `desiredVelocity` every frame after solve.
2. Opposite input allows penetration into moving platform:
   - add forced push lane + compensation pass.
3. Character flickers on edges/steps:
   - add grounded hysteresis + tiny grounded bias + vertical deadzone.
4. Rotating rectangular platform behaves like circular carry region:
   - use multi-sample support probe and point-based transform delta.
5. Character tilts with platform:
   - keep body upright; apply yaw-only carry.

## 16) Minimal order of operations (checklist)

1. Update support carry baseline.
2. Collect collision-based external/forced pushes.
3. Build desired move from input + carry + pushes + vertical.
4. Solve KCC pass 1.
5. Solve optional compensation pass 2.
6. Apply translation.
7. Apply upright rotation (+ optional support yaw).
8. Update grounded/jump state.
9. Clear one-frame intents.

If you keep this order, the controller remains predictable even in complex moving-platform scenes.

