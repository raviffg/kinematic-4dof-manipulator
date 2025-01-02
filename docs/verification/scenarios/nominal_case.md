# Nominal operating case and expected invariants

## Project context
four-degree-of-freedom serial manipulator kinematics and stall detection. This scenario exercises the project's core behavior: standard DH transforms, forward kinematics, damped-least-squares inverse kinematics, INA219 stall detection.

## Objective
Define a repeatable check for **nominal operating case and expected invariants** without assuming unverified hardware behavior or undocumented numerical limits.

## Relevant implementation
- Primary implementation: `src/dh_kinematics.cpp`
- Supporting implementation: `scripts/stall_detector.py`
- Main engineering concerns: joint limits, singularity damping, frame conventions, current baseline, median filtering, sustained stall flags.

## Procedure
1. Record the source revision, toolchain/runtime versions, and configuration.
2. Prepare a controlled input fixture and preserve its original bytes.
3. Execute the relevant function or command using the documented interface.
4. Capture raw input, output, diagnostics, and run metadata.
5. Repeat with the boundary or fault condition relevant to this scenario.
6. Compare the result with an independently reviewed expectation.

## Expected evidence
- Reproducible command or test harness invocation.
- Input fixture identifier and checksum.
- Observed output and diagnostic log.
- Pass/fail rationale tied to explicit acceptance criteria.

## Acceptance criteria
The behavior is deterministic for identical inputs, invalid data is handled according to the documented contract, and no safety-relevant failure is silently converted into a valid result.

## Open items
- Add measured acceptance limits when bench evidence is available.
- Link a checked-in fixture and automated test when implemented.
- Record unresolved assumptions rather than treating them as verified facts.
