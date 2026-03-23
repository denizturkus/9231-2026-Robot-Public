# Different Robot Checklist

Use this when cloning the codebase onto a second robot that has the same logic but different measured hardware constants.

Sim-only values are intentionally omitted.

## 1. Before first enable

- Verify every CAN ID and CAN bus assignment in `TunerConstants`, `FlywheelConstants`, `HoodConstants`, `TurretConstants`, `IntakeConstants`, `FeederConstants`, and `HopperConstants`.
- Verify every motor inversion before running mechanisms to hard stops.
- Verify the Pigeon ID and drivetrain CAN bus name.
- Verify Limelight names match the names configured on the cameras.

## 2. Drivetrain bring-up

- Measure wheel radius with real tread installed and update `TunerConstants.kWheelRadius`.
- Verify module X/Y locations match the actual frame geometry.
- Re-zero all four CANcoder offsets.
- Verify left/right drive inversion and steer inversion on each module.
- Re-characterize `kSpeedAt12Volts`.
- Re-tune steer gains, drive gains, and slip current.
- Re-measure robot mass, MOI, and wheel coefficient for PathPlanner-related constants.

## 3. Vision bring-up

- Measure `Constants.robotToTurret`.
- Measure `VisionConstants.robotToCamera0`.
- Measure the turret zero-angle camera transform in `VisionConstants.robotToCamera1`.
- Confirm the turret camera transform still looks correct across turret rotation in AdvantageScope.
- Tune `maxAmbiguity`, `maxZError`, and the per-camera rejection arrays.
- Tune `linearStdDevBaseline`, `angularStdDevBaseline`, and the per-camera trust factors.

## 4. Turret and hood bring-up

- Set the real turret calibration angle in `TurretConstants.kTurretCalibrationAngle`.
- Verify turret soft limits, gearbox reduction, inversion, and Motion Magic limits.
- Set the real hood calibration angle in `HoodConstants.kHoodCalibrationAngle`.
- Verify hood min/max angle, gearbox reduction, inversion, and Motion Magic limits.
- Retune turret and hood closed-loop gains on the real robot.

## 5. Flywheel and shot tuning

- Retune flywheel gains on the new robot.
- Set `FlywheelConstants.kMaxAllowedRPM` to the real safe limit for the new robot.
- Make sure `kMaxAllowedRPM` is compatible with `Constants.ShootOnTheMoveConstants.kFlywheelSpeedRpm`.
- Rebuild the hood angle table from real shot data.
- Rebuild the flywheel RPM table from real shot data.
- Rebuild the time-of-flight table from logged real shots.
- Revalidate `kMinDistanceMeters`, `kMaxDistanceMeters`, `kPhaseDelaySeconds`, and `kVelocityFilterWindowSeconds`.

## 6. Intake and indexing

- Verify intake arm calibration angle, open angle, closed angle, and soft limits.
- Verify intake roller voltages and arm gains.
- Verify feeder feed/unfeed voltages, inversion, and current limits.
- Verify hopper feed, fast-feed, and reverse voltages.
- Revisit current limits if the motors, gearing, or electrical package differ.

## 7. Final on-field validation

- Drive straight and confirm odometry stays straight.
- Rotate in place and confirm odometry heading matches reality.
- Confirm both Limelights agree with drivetrain pose when the robot is stationary.
- Confirm turret aiming still works with the turret camera active.
- Shoot from near, mid, and far distances and log hood angle, turret angle, flywheel RPM, and hit result.
- Re-run any constants that still produce drift, overshoot, or missed shots.

## Files with in-code TODO markers

- `src/main/java/frc/robot/Constants.java`
- `src/main/java/frc/robot/subsystems/drive/DriveSubsystem.java`
- `src/main/java/frc/robot/subsystems/drive/TunerConstants.java`
- `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`
- `src/main/java/frc/robot/subsystems/shooter/flywheel/FlywheelConstants.java`
- `src/main/java/frc/robot/subsystems/shooter/hood/HoodConstants.java`
- `src/main/java/frc/robot/subsystems/shooter/turret/TurretConstants.java`
- `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`
- `src/main/java/frc/robot/subsystems/indexer/feeder/FeederConstants.java`
- `src/main/java/frc/robot/subsystems/indexer/hopper/HopperConstants.java`
