// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.lib.Constants.BuildConstants;
import frc.lib.Constants.IntakeConstants;
import frc.lib.util.LoggedTunableNumber;

/** Add your docs here. */
public class IntakeIOKraken implements IntakeIO {

  private final TalonFX m_rotationMotor;
  private final TalonFX m_rotationMotorFollow;
  private final TalonFX m_intakingMotor;
  private final TalonFX m_intakingMotorFollow;

  private Slot0Configs m_rotationPIDConfigs;
  private Slot0Configs m_intakingPIDConfigs;
  private final TalonFXConfiguration m_rotationMotorConfig;
  private final TalonFXConfiguration m_intakingMotorConfig;
  private final VelocityTorqueCurrentFOC m_requestedVelocity;
  private final Follower followercontrol;
  private final Follower rotationFollowerControl;
  private final DynamicMotionMagicVoltage m_requestedPosition;
  private double m_requestedAngleDegrees = 0.0;
  private final StatusSignal<Angle> m_rotationPosition;
  private final StatusSignal<AngularVelocity> m_rotationVelocity;
  private final StatusSignal<Angle> m_rotationFollowerPosition;
  private final StatusSignal<AngularVelocity> m_intakingVelocity;
  private final Debouncer m_rotationMotorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer m_rotationFollowerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer m_intakingMotorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

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
    m_intakingMotorFollow = new TalonFX(IntakeConstants.intakingFollowMotorID, IntakeConstants.canbus);
    m_rotationMotor = new TalonFX(IntakeConstants.rotationMotorID, IntakeConstants.canbus);
    m_rotationMotorFollow =
        new TalonFX(IntakeConstants.rotationMotorFollowID, IntakeConstants.canbus);
    m_rotationPIDConfigs =
        new Slot0Configs()
            .withKP(rotationkp.get())
            .withKI(rotationki.get())
            .withKD(rotationkd.get())
            .withKV(IntakeConstants.angleMotorkv)
            .withKA(IntakeConstants.angleMotorka)
            .withKG(IntakeConstants.angleMotorkg).withGravityType(GravityTypeValue.Arm_Cosine);
     m_intakingPIDConfigs =
        new Slot0Configs()
            .withKP(IntakeConstants.intakingMotorkp)
            .withKI(IntakeConstants.intakingMotorki)
            .withKD(IntakeConstants.intakingMotorkd)
            .withKV(IntakeConstants.intakingMotorkv)
            .withKA(IntakeConstants.intakingMotorka)
            .withKS(IntakeConstants.intakingMotorks);

    // .withGravityType(GravityTypeValue.Arm_Cosine);
    m_rotationMotorConfig = new TalonFXConfiguration();
    m_intakingMotorConfig = new TalonFXConfiguration();
    m_rotationMotorConfig.Slot0 = m_rotationPIDConfigs;
    m_rotationMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.angleMotionMagicCruiseVelocityRotationsPerSecond;
    m_rotationMotorConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.angleMotionMagicAccelerationDegreesPerSecondSquared / 360.0;
    m_rotationMotorConfig.MotionMagic.MotionMagicJerk =
        IntakeConstants.angleMotionMagicJerkDegreesPerSecondCubed / 360.0;
    m_intakingMotorConfig.Slot0 = m_intakingPIDConfigs;
    m_rotationMotorConfig.Feedback.SensorToMechanismRatio = rotationGearRatio.get();
    m_rotationMotor.getConfigurator().apply(m_rotationMotorConfig);
    m_rotationMotorFollow.getConfigurator().apply(m_rotationMotorConfig);
    m_intakingMotor.getConfigurator().apply(m_intakingMotorConfig);
    m_intakingMotorFollow.getConfigurator().apply(m_intakingMotorConfig);
    m_requestedPosition = new DynamicMotionMagicVoltage(0, IntakeConstants.angleMotionMagicCruiseVelocityRotationsPerSecond, IntakeConstants.angleMotionMagicAccelerationDegreesPerSecondSquared);
    m_requestedVelocity = new VelocityTorqueCurrentFOC(0).withSlot(0);
    followercontrol = new Follower(IntakeConstants.intakingMotorID, MotorAlignmentValue.Opposed);
    rotationFollowerControl =
        new Follower(IntakeConstants.rotationMotorID, MotorAlignmentValue.Opposed);
    m_intakingMotorFollow.setControl(followercontrol);
    m_rotationMotorFollow.setControl(rotationFollowerControl);

    m_rotationPosition = m_rotationMotor.getPosition();
    m_rotationVelocity = m_rotationMotor.getVelocity();
    m_rotationFollowerPosition = m_rotationMotorFollow.getPosition();
    m_intakingVelocity = m_intakingMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, m_rotationPosition, m_rotationVelocity, m_rotationFollowerPosition, m_intakingVelocity);
    ParentDevice.optimizeBusUtilizationForAll(
        m_rotationMotor, m_rotationMotorFollow, m_intakingMotor);

    resetEncoder();
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
    m_requestedAngleDegrees = angleDegrees;
    m_rotationMotor.setControl(
        m_requestedPosition.withPosition(angleDegrees / 360).withEnableFOC(true));
  }

  public double getAngle() {
    return m_rotationPosition.getValueAsDouble() * 360.0;
  }

  public void setIntakeSpeed(double speedInRPS) {
    m_intakingMotor.setControl(m_requestedVelocity.withVelocity(speedInRPS));
    //m_intakingMotor.set(speedInRPS);
  }

  public void stopIntake() {

    m_intakingMotor.stopMotor();
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
            m_rotationPIDConfigs =
                new Slot0Configs()
                    .withKP(rotationkp.get())
                    .withKI(rotationki.get())
                    .withKD(rotationkd.get());
            m_rotationMotor.getConfigurator().apply(m_rotationPIDConfigs);
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

  public void jostleIntake() {
    m_requestedPosition.Velocity = IntakeConstants.jostleIntakeVelocity;
    setAngle(IntakeConstants.rotationUpDegrees + 15);
  }

  public void resetMotionMagic() {
    m_requestedPosition.Velocity = IntakeConstants.angleMotionMagicCruiseVelocityRotationsPerSecond;
  }

  public void updateInputs(IntakeIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            m_rotationPosition, m_rotationVelocity, m_rotationFollowerPosition, m_intakingVelocity);

    inputs.rotationMotorConnected = m_rotationMotorConnectedDebounce.calculate(status.isOK());
    inputs.intakingMotorConnected = m_intakingMotorConnectedDebounce.calculate(status.isOK());
    inputs.rotationMotorFollowerConnected =
        m_rotationFollowerConnectedDebounce.calculate(status.isOK());
    inputs.rotationDegrees = m_rotationPosition.getValueAsDouble() * 360.0;
    inputs.rotationPrimaryDegrees = m_rotationPosition.getValueAsDouble() * 360.0;
    inputs.rotationFollowerDegrees = m_rotationFollowerPosition.getValueAsDouble() * 360.0;
    inputs.rotationSpeedDegreesPerSecond = m_rotationVelocity.getValueAsDouble() * 360.0;
    inputs.rotationSetpointDegrees = m_requestedAngleDegrees;
  }
}
