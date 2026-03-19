// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants.GenericConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.targeting.ShootTargetIO;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final double MAX_RPM = 3600.0;
  private static final double[][] DISTANCE_TO_RPM_POINTS = {
    {3.11, 1825.0},
    {3.84, 2030.0},
    {2.35, 1800.0}
  };

  private ShooterIO shooter;
  private Drive drive;
  private IndexerIO indexer;
  private PhaseshiftIO phaseshift;
  private PhaseshiftIOInputsAutoLogged phaseshiftInputs;
  private ShootTargetIO shootTarget;

  /** FIX DO NOT WANT TO IMPORT A WHOLE DRIVE */
  public Shooter(
      ShooterIO shooter,
      Drive drive,
      IndexerIO indexer,
      PhaseshiftIO phaseshift,
      ShootTargetIO shootTarget) {
    this.shooter = shooter;
    this.drive = drive;
    this.indexer = indexer;
    this.phaseshift = phaseshift;
    this.shootTarget = shootTarget;
    this.phaseshiftInputs = new PhaseshiftIOInputsAutoLogged();

    // Distance (meters) -> RPM calibration points for quadratic interpolation
    // TODO: tune these values with real testing
    shootTarget.setTarget(GenericConstants.HUB_POSE3D, true);
  }

  @Override
  public void periodic() {
    // phaseshift.updateInputs(phaseshiftInputs);
    // shooter.updateInputs(null);
    // indexer.updateInputs(null);

    Logger.recordOutput("Shooter/DistanceToTargetMeters", getDistanceToTarget());
  }

  /** Returns the distance in meters from the robot to the current shoot target. */
  public double getDistanceToTarget() {
    Pose2d robotPose = drive.getPose();
    Translation2d targetXY = new Translation2d(shootTarget.getTarget().getX(), shootTarget.getTarget().getY());

    Logger.recordOutput("Shooter/targetpost", shootTarget.getTarget());
    return robotPose.getTranslation().getDistance(targetXY);
  }

  /** Returns the quadratically interpolated RPM for the current distance to target, capped at MAX_RPM. */
  public double getLookupRpm() {
    double distance = getDistanceToTarget();
    double rpm = interpolateQuadratic(distance);
    return Math.min(rpm, MAX_RPM);
  }

  private double interpolateQuadratic(double distance) {
    double rpm = 0.0;

    for (int i = 0; i < DISTANCE_TO_RPM_POINTS.length; i++) {
      double xi = DISTANCE_TO_RPM_POINTS[i][0];
      double yi = DISTANCE_TO_RPM_POINTS[i][1];
      double basis = 1.0;

      for (int j = 0; j < DISTANCE_TO_RPM_POINTS.length; j++) {
        if (i == j) {
          continue;
        }

        double xj = DISTANCE_TO_RPM_POINTS[j][0];
        basis *= (distance - xj) / (xi - xj);
      }

      rpm += yi * basis;
    }

    return rpm;
  }

  /** Spins the shooter at the lookup-table RPM based on distance to the current target. */
  public void spinShooterFromLookup() {
    double rpm = getLookupRpm();
    shooter.setShooterSpeed(rpm / 60.0); // convert RPM to RPS
  }

  public void spinShooter(double speed) {
    shooter.setShooterSpeed(speed);
  }

  public void stopShooter() {
    shooter.stopShooter();
  }

  public void setHoodAngle(double hoodAngle) {
    shooter.setHoodAngle(hoodAngle);
  }

  public void setIndexerSpeed(double indexerSpeedInRPS) {
    indexer.setIndexerSpeed(indexerSpeedInRPS);
  }

  public Drive getDrive() {
    return drive;
  }

  public ShootTargetIO getShootTarget() {
    return shootTarget;
  }
  public double getShooterVelocity() {
    return shooter.getVelocity();
  }

  public void setTarget(Translation3d target) {
    shootTarget.setTarget(target, true);
  }
}
