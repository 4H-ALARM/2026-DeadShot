// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.targeting.ShootTargetIO;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final double MAX_RPM = 3600.0;

  private ShooterIO shooter;
  private Drive drive;
  private IndexerIO indexer;
  private PhaseshiftIO phaseshift;
  private PhaseshiftIOInputsAutoLogged phaseshiftInputs;
  private ShootTargetIO shootTarget;

  // Distance (meters) -> RPM lookup table with linear interpolation
  private final InterpolatingDoubleTreeMap distanceToRpmTable = new InterpolatingDoubleTreeMap();

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

    // Populate lookup table: distance (meters) -> RPM
    // TODO: tune these values with real testing
    distanceToRpmTable.put(1.0, 1500.0);
    distanceToRpmTable.put(2.0, 1750.0);
    distanceToRpmTable.put(3.0, 2500.0);
    distanceToRpmTable.put(4.0, 3000.0);
    distanceToRpmTable.put(5.0, 3400.0);
    distanceToRpmTable.put(6.0, 3600.0);
  }

  @Override
  public void periodic() {
    phaseshift.updateInputs(phaseshiftInputs);
    shooter.updateInputs(null);
    indexer.updateInputs(null);

    Logger.recordOutput("Shooter/DistanceToTargetMeters", getDistanceToTarget());
  }

  /** Returns the distance in meters from the robot to the current shoot target. */
  public double getDistanceToTarget() {
    Pose2d robotPose = drive.getPose();
    Translation2d targetXY =
        new Translation2d(shootTarget.getTarget().getX(), shootTarget.getTarget().getY());
    return robotPose.getTranslation().getDistance(targetXY);
  }

  /** Returns the interpolated RPM for the current distance to target, capped at MAX_RPM. */
  public double getLookupRpm() {
    double distance = getDistanceToTarget();
    double rpm = distanceToRpmTable.get(distance);
    return Math.min(rpm, MAX_RPM);
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
}
