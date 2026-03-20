// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.targeting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.AllianceFlipUtil;

import org.littletonrobotics.junction.AutoLog;

public class ShootTargetIO extends SubsystemBase {
  final Translation3d m_defaultTarget;
  final boolean m_defaultTargetAvailable;
  Translation3d m_target;
  boolean m_targetAvailable;

  @AutoLog
  public static class ShootTargetIOInputs {
    public Translation3d target;
    public boolean targetAvailable;
  }

  /**
   * Creates a new ShootTarget.
   *
   * <p>some targets like hub will be available or unavailable based on which specific phaseshift is
   * active./*
   */
  public ShootTargetIO(Translation3d defaultTarget, boolean targetAvailable) {
    this.m_defaultTarget = defaultTarget;
    this.m_defaultTargetAvailable = targetAvailable;

    this.resetTarget();
  }

  public void resetTarget() {
    setTarget(m_defaultTarget, m_defaultTargetAvailable);
  }

  public void setTarget(Translation3d target, boolean targetAvailable) {
    this.m_target = target;
    this.m_targetAvailable = targetAvailable;
  }

  public Translation3d getTarget() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      Translation2d flippedtranslation = AllianceFlipUtil.apply(new Translation2d(this.m_target.getX(), this.m_target.getY()));

      return new Translation3d(flippedtranslation.getX(), flippedtranslation.getY(), this.m_target.getZ());
    }

    return this.m_target;

  }

  public boolean isTargetAvailable() {
    return m_targetAvailable;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
