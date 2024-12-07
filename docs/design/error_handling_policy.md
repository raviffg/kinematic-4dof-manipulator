# Error handling policy

## Scope
This note records the design considerations for four-degree-of-freedom serial manipulator kinematics and stall detection.

## Current project context
The repository currently implements standard DH transforms, forward kinematics, damped-least-squares inverse kinematics, INA219 stall detection. The main code paths are `src/dh_kinematics.cpp` and `scripts/stall_detector.py`. The principal engineering concerns are joint limits, singularity damping, frame conventions, current baseline, median filtering, sustained stall flags.

## Design questions
- What inputs, units, coordinate conventions, and preconditions does this part of the system require?
- Which state is authoritative, and which values are derived?
- How are invalid, stale, incomplete, or physically implausible inputs represented?
- Which assumptions are supported by source code or measured evidence, and which remain provisional?
- What observable evidence allows a reviewer to reproduce the behavior?

## Recommended design record
Document the selected approach, alternatives considered, rationale, expected failure modes, and any measurable trade-offs. Keep numerical constants linked to source or calibration evidence; do not copy unexplained values into a second source of truth.

## Verification
Pair this design note with the relevant scenarios under `docs/verification/scenarios/`. A design decision is not considered verified merely because the code compiles.

## Status
Working reference. Confirm implementation details against the source before using this note as a hardware, clinical, or production specification.
