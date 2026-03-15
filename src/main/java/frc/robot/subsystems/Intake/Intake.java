// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO m_intakeIO;

  private IntakeIOInputs m_inputs;

  public Intake(IntakeIO intake) {
    this.m_intakeIO = intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void resetEncoder() {
    m_intakeIO.resetEncoder();
  }

  /// don't use for regular code, no automatic stop.
  public void changeAngleTest(double speed) {
    m_intakeIO.changeAngleTest(speed);
    // dont use for regular code, no automatic stop
  }

  public void setAngle(double angleDegrees) {
    m_intakeIO.setAngle(angleDegrees);
  }

  public void setIntakeSpeed(double speed) {
    m_intakeIO.setIntakeSpeed(speed);
  }

  public void stopIntake() {
    m_intakeIO.stopIntake();
  }

  public void updateInputs() {
    m_intakeIO.updateInputs(m_inputs);
  }
}
