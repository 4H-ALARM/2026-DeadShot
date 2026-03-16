// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Shooter extends SubsystemBase {

  private ShooterIO shooter;
  private Drive drive;
  private IndexerIO indexer;

  /** FIX DO NOT WANT TO IMPORT A WHOLE DRIVE */
  public Shooter(ShooterIO shooter, Drive drive, IndexerIO indexer) {
    this.shooter = shooter;
    this.drive = drive;
    this.indexer = indexer;
  }

  @Override
  public void periodic() {
    // this.turret.updateInputs(turretInputs);
    // this.turret.turretPeriodic(drive.getPose());
  }

  public void spinShooter(double speed) {
    shooter.setShooterSpeed(speed);
  }

  public void spinHood(double speed) {
    shooter.setHoodSpeed(speed);
  }

  public void setIndexerSpeed(double indexerSpeed) {
    indexer.setIndexerSpeed(indexerSpeed);
  }
}
