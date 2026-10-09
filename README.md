# Rapier-Kinematic-Character-Controller(kcc)

**A stable kinematic character controller for Rapier with moving platforms, rotation support and reliable grounding.**

---

## Overview

This repository provides a **practical, working implementation** of a kinematic character controller built on top of Rapier.

It is extracted from a real engine and designed to be **close to plug-and-play**, requiring only minimal adaptation depending on your setup.

---

## Features

* Stable grounding (no flicker on edges or steps)
* Autostep and slope handling
* Moving platform carry (translation + rotation)
* External push handling (including rotating platforms)
* Prevention of penetration when input opposes movement
* Compensation pass for locked/moving obstacles
* Upright character behavior (no unwanted tilt)
* Grounded hysteresis and anti-jitter fixes
* Surface friction that changes acceleration and stopping on supported ground
* Character mass controls whether the KCC pushes dynamic bodies
* Sensor exclusion during movement queries

---

## Why this exists

Most Rapier character controller examples only cover basic movement and break in real scenarios.

This implementation focuses on real-world behavior:

* Moving platforms (translation + rotation)
* Rotating obstacles pushing the character
* Stable stepping and slope handling
* Consistent behavior under conflicting forces

---

## Usage

This is a **working reference implementation**, not just theory.

### Basic integration

1. Plug the controller into your update loop
2. Write each character's `_kccDesiredVelocity` every frame
3. Call `beforePhysicsStep(dt)`, `updatePlatformCarryState()` and `solveCharacters(dt)` before stepping Rapier
4. After the physics step, call `collectCollisionPushes()` and `updateResolvedSupport()`

### Notes

* Some adaptation may be required depending on your engine structure
* You control movement through high-level input, not direct transforms
* Set `go.mass` to a positive value to push dynamic bodies, or `0` to disable that behavior
* Set `go.characterSurfaceFrictionScale` to tune how strongly support friction affects movement

**Tested with:**

* three.js + Rapier

---

## Structure

* `RAPIER_KINEMATIC_CHARACTER_CONTROLLER.md`
  Full explanation and design notes

* `RAPIER_KCC.js`
  Actual implementation extracted from a real engine

---

## Key Concepts

* Movement is **intent-driven**, not physics-driven
* Platforms require **explicit carry logic**
* External forces are **manually accumulated**
* Stability comes from **multiple small corrections working together**
