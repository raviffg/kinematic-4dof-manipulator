# kinematic-4dof-manipulator

4-DOF serial arm kinematics plus a bench harness for stall detection.

## Layout

    include/dh_chain.hpp        DH structs, 4x4 transforms, joint config
    src/dh_kinematics.cpp       forward kinematics + DLS inverse kinematics
    scripts/stall_detector.py   INA219 shunt log analyser
    data/ina219_shunt_capture.csv

## Build

    make

## Run

    ./build/dh_kinematics
    python scripts/stall_detector.py data/ina219_shunt_capture.csv

The IK solver uses `dq = J^T (J J^T + lambda^2 I)^-1 e` with position-only
rows. Joint limits are clamped every iteration and the damping term grows when
the step size stalls, which is what keeps the elbow singularity from throwing
NaN into the trajectory buffer.

Stall detector expects the CSV columns `timestamp_s, joint_id, bus_voltage_v,
shunt_current_ma, stall_flag`. Median filter of 5, EMA alpha 0.25, threshold
is idle baseline + 180 mA sustained for 3 samples.

Tested on g++ 13 / Python 3.11. No external dependencies.
