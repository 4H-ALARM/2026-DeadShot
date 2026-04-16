/*
 * ProjectileSimulator.java - RK4 projectile physics with drag and Magnus lift
 *
 * MIT License
 *
 * Copyright (c) 2026 FRC Team 5962 perSEVERE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
 */
package frc.lib.firecontrol;

import java.util.ArrayList;
import java.util.List;

/**
 * Simulates a ball in the vertical plane (x, z) with drag and Magnus lift.
 *
 * <p>For each distance, binary-searches RPM until the ball reaches target height, then builds a
 * lookup table of distance -> RPM and TOF.
 */
public class ProjectileSimulator {
  /** Physical and solver parameters. */
  public record SimParameters(
      double ballMassKg,
      double ballDiameterM,
      double dragCoeff,
      double magnusCoeff,
      double airDensity,
      double exitHeightM,
      double wheelDiameterM,
      double targetHeightM,
      double slipFactor,
      double fixedLaunchAngleDeg,
      double dt,
      double rpmMin,
      double rpmMax,
      int binarySearchIters,
      double maxSimTime) {}

  public record TrajectoryResult(
      double zAtTarget, double tof, boolean reachedTarget, double maxHeight, double apexX) {}

  /** One LUT row. */
  public record LUTEntry(double distanceM, double rpm, double tof, boolean reachable) {}

  /** Generated LUT and metadata. */
  public record GeneratedLUT(
      List<LUTEntry> entries,
      SimParameters params,
      int reachableCount,
      int unreachableCount,
      double maxRangeM,
      long generationTimeMs) {}

  private static final double GRAVITY_MPS2 = 9.81;
  private static final double LUT_MIN_DISTANCE_M = 0.50;
  private static final double LUT_MAX_DISTANCE_M = 6.00;
  private static final double LUT_STEP_M = 0.05;

  private final SimParameters params;

  // Precomputed aero constants.
  private final double kDrag;
  private final double kMagnus;
  private double magnusSign = 1.0;

  public ProjectileSimulator(SimParameters params) {
    this(params, 1.0);
  }

  public ProjectileSimulator(SimParameters params, double magnusSign) {
    this.params = params;
    double area = Math.PI * (params.ballDiameterM() / 2.0) * (params.ballDiameterM() / 2.0);
    this.kDrag = (params.airDensity() * params.dragCoeff() * area) / (2.0 * params.ballMassKg());
    this.kMagnus =
        (params.airDensity() * params.magnusCoeff() * area) / (2.0 * params.ballMassKg());
    setMagnusSign(magnusSign);
  }

  public void setMagnusSign(double magnusSign) {
    this.magnusSign = Math.signum(magnusSign) == 0.0 ? 1.0 : Math.signum(magnusSign);
  }

  /** Convert flywheel RPM to ball exit velocity (m/s). */
  public static double rpmToExitVelocity(double rpm, double wheelDiameterM, double slipFactor) {
    return slipFactor * rpm * Math.PI * wheelDiameterM / 60.0;
  }

  /** Convert ball exit velocity (m/s) to flywheel RPM. */
  public static double exitVelocityToRPM(
      double exitVelocityMps, double wheelDiameterM, double slipFactor) {
    return exitVelocityMps * 60.0 / (Math.PI * wheelDiameterM * Math.max(slipFactor, 1e-6));
  }

  /** RPM to ball exit velocity (m/s), including slip factor. */
  public double exitVelocity(double rpm) {
    return rpmToExitVelocity(rpm, params.wheelDiameterM(), params.slipFactor());
  }

  /**
   * Simulate at a given RPM and sample trajectory at targetDistanceM.
   */
  public TrajectoryResult simulate(double rpm, double targetDistanceM) {
    return simulateAtAngle(rpm, targetDistanceM, params.fixedLaunchAngleDeg());
  }

  private TrajectoryResult simulateAtAngle(
      double rpm, double targetDistanceM, double launchAngleDeg) {
    double v0 = exitVelocity(rpm);
    double launchRad = Math.toRadians(launchAngleDeg);
    double vx = v0 * Math.cos(launchRad);
    double vz = v0 * Math.sin(launchRad);

    double x = 0;
    double z = params.exitHeightM();

    double dt = params.dt();
    double maxHeight = z;
    double apexX = 0;

    double t = 0;
    double maxTime = params.maxSimTime();

    while (t < maxTime) {
      // RK4 step.
      double[] state = {x, z, vx, vz};
      double[] k1 = derivatives(state);
      double[] s2 = addScaled(state, k1, dt / 2.0);
      double[] k2 = derivatives(s2);
      double[] s3 = addScaled(state, k2, dt / 2.0);
      double[] k3 = derivatives(s3);
      double[] s4 = addScaled(state, k3, dt);
      double[] k4 = derivatives(s4);

      x += dt / 6.0 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
      z += dt / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
      vx += dt / 6.0 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
      vz += dt / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
      t += dt;

      if (z > maxHeight) {
        maxHeight = z;
        apexX = x;
      }

      if (x >= targetDistanceM) {
        // Same approximation as upstream source.
        double prevX = x - vx * dt;
        double prevZ = z - vz * dt;
        double frac = (targetDistanceM - prevX) / (x - prevX);
        double zAtTarget = prevZ + frac * (z - prevZ);
        double tofAtTarget = t - dt + frac * dt;
        return new TrajectoryResult(zAtTarget, tofAtTarget, true, maxHeight, apexX);
      }

      if (z < 0) {
        return new TrajectoryResult(0, t, false, maxHeight, apexX);
      }
    }

    return new TrajectoryResult(0, maxTime, false, maxHeight, apexX);
  }

  // state = [x, z, vx, vz]
  // ax = -kDrag * |v| * vx
  // az = -g - kDrag * |v| * vz + kMagnus * |v|^2
  // (positive kMagnus is upward lift for backspin)
  private double[] derivatives(double[] state) {
    double svx = state[2];
    double svz = state[3];
    double speed = Math.hypot(svx, svz);

    double ax = -kDrag * speed * svx;
    double az = -GRAVITY_MPS2 - kDrag * speed * svz + magnusSign * kMagnus * speed * speed;

    return new double[] {svx, svz, ax, az};
  }

  private static double[] addScaled(double[] base, double[] delta, double scale) {
    return new double[] {
      base[0] + delta[0] * scale,
      base[1] + delta[1] * scale,
      base[2] + delta[2] * scale,
      base[3] + delta[3] * scale
    };
  }

  /** Binary-search RPM to hit target height at distanceM. */
  public LUTEntry findRPMForDistance(double distanceM) {
    return findRPMForDistance(distanceM, params.fixedLaunchAngleDeg());
  }

  private LUTEntry findRPMForDistance(double distanceM, double launchAngleDeg) {
    double heightTolerance = 0.02;
    double lo = params.rpmMin();
    double hi = params.rpmMax();

    TrajectoryResult maxCheck = simulateAtAngle(hi, distanceM, launchAngleDeg);
    if (!maxCheck.reachedTarget()) {
      return new LUTEntry(distanceM, 0, 0, false);
    }

    double bestRpm = hi;
    double bestTof = maxCheck.tof();
    double bestError = Math.abs(maxCheck.zAtTarget() - params.targetHeightM());

    for (int i = 0; i < params.binarySearchIters(); i++) {
      double mid = (lo + hi) / 2.0;
      TrajectoryResult result = simulateAtAngle(mid, distanceM, launchAngleDeg);

      if (!result.reachedTarget()) {
        lo = mid;
        continue;
      }

      double error = result.zAtTarget() - params.targetHeightM();
      double absError = Math.abs(error);

      if (absError < bestError) {
        bestRpm = mid;
        bestTof = result.tof();
        bestError = absError;
      }

      if (absError < heightTolerance) {
        return new LUTEntry(distanceM, mid, result.tof(), true);
      }

      if (error > 0) {
        hi = mid;
      } else {
        lo = mid;
      }
    }

    return new LUTEntry(distanceM, bestRpm, bestTof, bestError < 0.10);
  }

  /** Generate LUT from 0.50m to 6.00m in 0.05m steps (111 entries). */
  public GeneratedLUT generateLUT() {
    return generateLUT(LUT_MIN_DISTANCE_M, LUT_MAX_DISTANCE_M, LUT_STEP_M);
  }

  /** Generate LUT over a custom distance range and step. */
  public GeneratedLUT generateLUT(double minDistanceM, double maxDistanceM, double stepM) {
    long startMs = System.currentTimeMillis();

    List<LUTEntry> entries = new ArrayList<>();
    int reachable = 0;
    int unreachable = 0;
    double maxRange = 0;

    int numSteps = (int) Math.round((maxDistanceM - minDistanceM) / stepM);
    for (int i = 0; i <= numSteps; i++) {
      double distance = minDistanceM + i * stepM;
      distance = Math.round(distance * 100.0) / 100.0;

      LUTEntry entry = findRPMForDistance(distance);
      entries.add(entry);

      if (entry.reachable()) {
        reachable++;
        maxRange = distance;
      } else {
        unreachable++;
      }
    }

    long elapsed = System.currentTimeMillis() - startMs;
    return new GeneratedLUT(entries, params, reachable, unreachable, maxRange, elapsed);
  }

  /**
   * Generate a variable-angle LUT by sweeping launch angle and selecting reachable shots that:
   *
   * <p>1) minimize hood movement from the previous distance point, 2) prefer steeper launch angles
   * (more vertical shot path), and 3) use lower RPM as a final tiebreaker.
   */
  public ShotLUT generateVariableAngleShotLUT(
      double minAngleDeg, double maxAngleDeg, double angleStepDeg) {
    ShotLUT shotLut = new ShotLUT();
    int numSteps = (int) Math.round((LUT_MAX_DISTANCE_M - LUT_MIN_DISTANCE_M) / LUT_STEP_M);
    Double previousAngleDeg = null;

    for (int i = 0; i <= numSteps; i++) {
      double distance = LUT_MIN_DISTANCE_M + i * LUT_STEP_M;
      distance = Math.round(distance * 100.0) / 100.0;

      LUTEntry bestEntry = null;
      double bestAngle = minAngleDeg;
      double bestHoodMovement = Double.POSITIVE_INFINITY;

      int angleSteps = (int) Math.round((maxAngleDeg - minAngleDeg) / angleStepDeg);
      for (int j = 0; j <= angleSteps; j++) {
        double angle = minAngleDeg + j * angleStepDeg;
        LUTEntry candidate = findRPMForDistance(distance, angle);
        if (!candidate.reachable()) {
          continue;
        }

        double hoodMovement =
            previousAngleDeg == null ? 0.0 : Math.abs(angle - previousAngleDeg.doubleValue());

        if (isPreferredAngleCandidate(
            candidate, angle, hoodMovement, bestEntry, bestAngle, bestHoodMovement)) {
          bestEntry = candidate;
          bestAngle = angle;
          bestHoodMovement = hoodMovement;
        }
      }

      if (bestEntry != null) {
        shotLut.put(distance, bestEntry.rpm(), bestAngle, bestEntry.tof());
        previousAngleDeg = bestAngle;
      }
    }

    return shotLut;
  }

  private static boolean isPreferredAngleCandidate(
      LUTEntry candidate,
      double candidateAngleDeg,
      double candidateHoodMovementDeg,
      LUTEntry currentBest,
      double currentBestAngleDeg,
      double currentBestHoodMovementDeg) {
    final double epsilon = 1e-6;

    if (currentBest == null) {
      return true;
    }

    if (candidateHoodMovementDeg + epsilon < currentBestHoodMovementDeg) {
      return true;
    }
    if (Math.abs(candidateHoodMovementDeg - currentBestHoodMovementDeg) > epsilon) {
      return false;
    }

    if (candidateAngleDeg > currentBestAngleDeg + epsilon) {
      return true;
    }
    if (Math.abs(candidateAngleDeg - currentBestAngleDeg) > epsilon) {
      return false;
    }

    return candidate.rpm() < currentBest.rpm();
  }

  // Package-private for testing.
  double getKDrag() {
    return kDrag;
  }

  double getKMagnus() {
    return kMagnus;
  }
}
