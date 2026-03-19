// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.Constants.BuildConstants;
import frc.lib.Constants.IntakeConstants;
import frc.lib.util.LoggedTunableNumber;

/** Add your docs here. */
public class IntakeIOKraken implements IntakeIO {

  private final TalonFX m_rotationMotor;
  private final TalonFX m_rotationMotorFollow;
  private final TalonFX m_intakingMotor;

  private Slot0Configs m_pIDConfigs;
  private final TalonFXConfiguration m_rotationMotorConfig;
  private final PositionVoltage m_requestedPosition;
  private double m_requestedAngleDegrees = 0.0;

  private final LoggedTunableNumber rotationkp =
      new LoggedTunableNumber("Intake/Rotation/kp", IntakeConstants.angleMotorkp);
  private final LoggedTunableNumber rotationki =
      new LoggedTunableNumber("Intake/Rotation/ki", IntakeConstants.angleMotorki);
  private final LoggedTunableNumber rotationkd =
      new LoggedTunableNumber("Intake/Rotation/kd", IntakeConstants.angleMotorkd);
  private final LoggedTunableNumber rotationGearRatio =
      new LoggedTunableNumber("Intake/Rotation/gearRatio", IntakeConstants.rotationGearRatio);

  public IntakeIOKraken() {
    m_intakingMotor = new TalonFX(IntakeConstants.intakingMotorID, IntakeConstants.canbus);
    m_rotationMotor = new TalonFX(IntakeConstants.rotationMotorID, IntakeConstants.canbus);
    m_rotationMotorFollow =
        new TalonFX(IntakeConstants.rotationMotorFollowID, IntakeConstants.canbus);
    // m_rotationMotorFollow.setControl(
    //     new Follower(IntakeConstants.rotationMotorID, MotorAlignmentValue.Opposed));
    m_pIDConfigs =
        new Slot0Configs()
            .withKP(rotationkp.get())
            .withKI(rotationki.get())
            .withKD(rotationkd.get())
            .withKV(IntakeConstants.angleMotorkv)
            .withKA(IntakeConstants.angleMotorka)
            .withKG(IntakeConstants.angleMotorkg);
    // .withGravityType(GravityTypeValue.Arm_Cosine);
    m_rotationMotorConfig = new TalonFXConfiguration();
    m_rotationMotorConfig.Slot0 = m_pIDConfigs;
    m_rotationMotorConfig.Feedback.SensorToMechanismRatio = rotationGearRatio.get();
    m_rotationMotor.getConfigurator().apply(m_rotationMotorConfig);
    m_rotationMotorFollow.getConfigurator().apply(m_rotationMotorConfig);
    m_requestedPosition = new PositionVoltage(0).withSlot(0);
  }

  public void resetEncoder() {
    m_rotationMotor.setPosition(0);
    m_rotationMotorFollow.setPosition(0);
    m_requestedAngleDegrees = 0.0;
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
    m_rotationMotor.setControl(
        m_requestedPosition.withPosition(angleDegrees / 360).withEnableFOC(true));
    m_rotationMotorFollow.setControl(
        m_requestedPosition.withPosition(-angleDegrees / 360).withEnableFOC(true));
  }

  public double getAngle() {
    return m_rotationMotor.getPosition().getValueAsDouble() * IntakeConstants.rotationGearRatio / 360;
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

  public boolean isIntakeUp() {
    return getAngle() < IntakeConstants.rotationDownDegrees / 4;
  }

  @Override
  public void updateTuningValues() {
    if (BuildConstants.TUNING) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () -> {
            m_pIDConfigs =
                new Slot0Configs()
                    .withKP(rotationkp.get())
                    .withKI(rotationki.get())
                    .withKD(rotationkd.get());
            m_rotationMotor.getConfigurator().apply(m_pIDConfigs);
          },
          rotationkp,
          rotationki,
          rotationkd);

      LoggedTunableNumber.ifChanged(
          hashCode(),
          () -> {
            m_rotationMotorConfig.Feedback.SensorToMechanismRatio = rotationGearRatio.get();
            m_rotationMotor.getConfigurator().apply(m_rotationMotorConfig);
          },
          rotationGearRatio);
    }
  }

  public void updateInputs(IntakeIOInputs inputs) {

    inputs.rotationMotorConnected = m_rotationMotor.isConnected();
    inputs.intakingMotorConnected = m_intakingMotor.isConnected();
    inputs.rotationMotorFollowerConnected = m_rotationMotorFollow.isConnected();
    inputs.rotationDegrees = m_rotationMotor.getPosition().getValueAsDouble();
    inputs.rotationSpeedDegreesPerSecond = m_rotationMotor.getVelocity().getValueAsDouble();
    inputs.rotationSetpointDegrees = m_requestedAngleDegrees;
  }
}
