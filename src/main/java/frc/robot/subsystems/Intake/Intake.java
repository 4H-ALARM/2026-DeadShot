// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants.IntakeConstants;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.subsystems.Intake.IntakeIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO m_intakeIO;

  private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();
  private final LoggedTunableNumber m_upRotationDegrees =
      new LoggedTunableNumber("Intake/upRotationDegrees", IntakeConstants.rotationUpDegrees);
  private final LoggedTunableNumber m_downRotationDegrees =
      new LoggedTunableNumber("Intake/downRotationDegrees", IntakeConstants.rotationDownDegrees);
  private int disabledSampleCounter = 0;

  public Intake(IntakeIO intake) {
    this.m_intakeIO = intake;
  }

  @Override
  public void periodic() {
    // m_intakeIO.updateTuningValues();

    // Disabled mode does not need full-rate intake telemetry every loop.
    if (DriverStation.isDisabled() && (disabledSampleCounter++ % 3 != 0)) {
      return;
    }
    m_intakeIO.updateInputs(m_inputs);
    Logger.processInputs("Intake", m_inputs);
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

  public void setRotationUp() {
    setAngle(m_upRotationDegrees.get());
  }

  public void setRotationDown() {
    setAngle(m_downRotationDegrees.get());
  }
  public double getAngle() {
    return m_intakeIO.getAngle();
  }

  public void setIntakeSpeed(double speed) {
    m_intakeIO.setIntakeSpeed(speed);
  }

  public void stopIntake() {
    m_intakeIO.stopIntake();
  }

  public boolean isIntakeUp() {
    return m_intakeIO.isIntakeUp();
  }

  public double getRotationDegrees() {
    return m_inputs.rotationDegrees;
  }
}
