// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.lib.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOKraken implements IntakeIO {

  TalonFX m_rotationMotor;
  TalonFX m_rotationMotorFollow;
  TalonFX m_intakingMotor;
  TalonFX m_intakingMotorFollow;

  Slot0Configs m_pIDConfigs;
  PositionVoltage m_requestedVoltage;

  public IntakeIOKraken() {
    m_intakingMotor = new TalonFX(IntakeConstants.intakingMotorID, IntakeConstants.canbus);
    m_intakingMotorFollow = new TalonFX(IntakeConstants.intakingMotorFollowID, IntakeConstants.canbus);
    m_intakingMotorFollow.setControl(
      new Follower(IntakeConstants.intakingMotorID, MotorAlignmentValue.Opposed)
    );
    m_rotationMotor = new TalonFX(IntakeConstants.rotationMotorID, IntakeConstants.canbus);
    m_rotationMotorFollow = new TalonFX(IntakeConstants.rotationMotorFollowID, IntakeConstants.canbus);
    m_rotationMotorFollow.setControl(
        new Follower(IntakeConstants.rotationMotorID, MotorAlignmentValue.Opposed));
    m_pIDConfigs =
        new Slot0Configs()
            .withKP(IntakeConstants.angleMotorkp)
            .withKI(IntakeConstants.angleMotorki)
            .withKD(IntakeConstants.angleMotorkd);
    m_rotationMotor.getConfigurator().apply(m_pIDConfigs);
    m_requestedVoltage = new PositionVoltage(0).withSlot(0);
  }

  public void resetEncoder() {
    m_rotationMotor.setPosition(0);
  }

  public void changeAngleTest(double speed) {
    if (speed > 1) {
      speed = 1;
    }
    if (speed < -1) {
      speed = -1;
    }
    m_rotationMotor.set(speed);
  }

  public void setAngle(double angleDegrees) {
    // still need to find the angle before this will work;
    double motorRotations =
        angleDegrees * IntakeConstants.rotationGearRatio / 360;

    m_rotationMotor.setControl(m_requestedVoltage.withPosition(motorRotations));
  }

  public void setIntakeSpeed(double speed) {
    if (speed > 1) {
      speed = 1;
    }
    if (speed < -1) {
      speed = -1;
    }
    m_intakingMotor.set(speed);
  }

  public void stopIntake() {

    m_intakingMotor.set(0);
  }

  public void updateInputs(IntakeIOInputs inputs) {

    inputs.rotationMotorConnected = m_rotationMotor.isConnected();
    inputs.intakingMotorConnected = m_intakingMotor.isConnected();
    inputs.rotationMotorFollowerConnected = m_rotationMotorFollow.isConnected();

    // need to figure out how many encoder units equals one full rotation
  }
}
