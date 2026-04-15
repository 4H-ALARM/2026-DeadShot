// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.Constants.GenericConstants;
import frc.lib.Constants.ShooterConstants;
import frc.lib.enums.TargetEnum;
import frc.lib.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterIOKraken implements ShooterIO {
  private TalonFX topShooterMotorRight;
  private TalonFX topShooterMotorLeft;
  private TalonFX bottomShooterMotorRight;
  private TalonFX bottomShooterMotorLeft;

  private CANcoder hoodEncoder;

  private Slot0Configs shooterConfig;
  private MotionMagicConfigs shooterMotionMagicConfig;
  private TalonFXConfiguration shooterMotorConfig;

  private TargetEnum targetEnum;
  private Pose2d robotPose;
  private Pose2d turretPose;
  private Translation3d targetPose;

  private PositionVoltage hoodPositionVoltage;
  private VelocityTorqueCurrentFOC shooterVelocityVoltage;

  private final LoggedTunableNumber shooterkp =
      new LoggedTunableNumber("Shooter/ShotTuning/kp", ShooterConstants.shooterkp);
  private final LoggedTunableNumber shooterki =
      new LoggedTunableNumber("Shooter/ShotTuning/ki", ShooterConstants.shooterki);
  private final LoggedTunableNumber shooterkd =
      new LoggedTunableNumber("Shooter/ShotTuning/kd", ShooterConstants.shooterkd);
  private final LoggedTunableNumber shooterka =
      new LoggedTunableNumber("Shooter/ShotTuning/ka", ShooterConstants.shooterka);
  private final LoggedTunableNumber shooterks =
      new LoggedTunableNumber("Shooter/ShotTuning/ks", ShooterConstants.shooterks);
  private final LoggedTunableNumber shooterkv =
      new LoggedTunableNumber("Shooter/ShotTuning/kv", ShooterConstants.shooterkv);
  private final LoggedTunableNumber shooterMaxAccel =
      new LoggedTunableNumber("Shooter/ShotTuning/maxAccel", ShooterConstants.shooterMaxAccel);
  private final LoggedTunableNumber shooterMaxSpeed =
      new LoggedTunableNumber("Shooter/ShotTuning/maxSpeed", ShooterConstants.shooterMaxSpeed);
  private final LoggedTunableNumber shooterJerk =
      new LoggedTunableNumber("Shooter/ShotTuning/jerk", ShooterConstants.shooterJerk);
  private final LoggedTunableNumber shooterFF =
      new LoggedTunableNumber("Shooter/ShotTuning/shooterFF", ShooterConstants.shooterFF);


  public ShooterIOKraken() {

    topShooterMotorRight =
        new TalonFX(ShooterConstants.topShooterMotorRightID, ShooterConstants.shooterCanbus);
    topShooterMotorLeft =
        new TalonFX(ShooterConstants.topShooterMotorLeftID, ShooterConstants.shooterCanbus);
    topShooterMotorLeft.setControl(
        new Follower(ShooterConstants.topShooterMotorRightID, MotorAlignmentValue.Opposed));
    bottomShooterMotorRight =
        new TalonFX(ShooterConstants.bottomShooterMotorRightID, ShooterConstants.shooterCanbus);
    bottomShooterMotorRight.setControl(
        new Follower(ShooterConstants.topShooterMotorRightID, MotorAlignmentValue.Aligned));
    bottomShooterMotorLeft =
        new TalonFX(ShooterConstants.bottomShooterMotorLeftID, ShooterConstants.shooterCanbus);
    bottomShooterMotorLeft.setControl(
        new Follower(ShooterConstants.topShooterMotorRightID, MotorAlignmentValue.Opposed));
    hoodEncoder = new CANcoder(ShooterConstants.hoodEncoderID, ShooterConstants.shooterCanbus);

    shooterConfig =
        new Slot0Configs()
            .withKP(shooterkp.get())
            .withKI(shooterki.get())
            .withKD(shooterkd.get())
            .withKA(shooterka.get())
            .withKS(shooterks.get())
            .withKV(shooterkv.get());

    shooterMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(shooterMaxAccel.get())
            .withMotionMagicCruiseVelocity(shooterMaxSpeed.get())
            .withMotionMagicJerk(shooterJerk.get());

    CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(45)
      .withStatorCurrentLimitEnable(true)
      .withSupplyCurrentLimit(50)
      .withSupplyCurrentLimitEnable(true);

    TorqueCurrentConfigs shooterTorqueCurrentConfigs = new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(45).withPeakReverseTorqueCurrent(-45);


    shooterVelocityVoltage = new VelocityTorqueCurrentFOC(0).withSlot(0);
    shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.Slot0 = shooterConfig;
    shooterMotorConfig.CurrentLimits = shooterCurrentLimitsConfigs;
    shooterMotorConfig.TorqueCurrent = shooterTorqueCurrentConfigs;

    // shooterMotorConfig.MotionMagic = shooterMotionMagicConfig;

    topShooterMotorRight.getConfigurator().apply(shooterMotorConfig);
  }

  @Override
  public void updateTuningValues() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          shooterConfig =
              new Slot0Configs()
                  .withKP(shooterkp.get())
                  .withKI(shooterki.get())
                  .withKD(shooterkd.get())
                  .withKA(shooterka.get())
                  .withKS(shooterks.get())
                  .withKV(shooterkv.get());
          topShooterMotorRight.getConfigurator().apply(shooterConfig);
        },
        shooterkp,
        shooterki,
        shooterkd,
        shooterka,
        shooterks,
        shooterkv);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          shooterMotionMagicConfig =
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(shooterMaxAccel.get())
                  .withMotionMagicCruiseVelocity(shooterMaxSpeed.get())
                  .withMotionMagicJerk(shooterJerk.get());
          topShooterMotorRight.getConfigurator().apply(shooterMotionMagicConfig);
        },
        shooterMaxAccel,
        shooterMaxSpeed,
        shooterJerk);


  }

  @Override
  public void setShooterSpeed(double speed) {
    // topShooterMotorRight.set(speed);
    topShooterMotorRight.setControl(
        shooterVelocityVoltage.withVelocity(speed).withFeedForward(shooterFF.get()).withUpdateFreqHz(1000));
  }

  @Override
  public void changeTarget(TargetEnum target) {
    targetEnum = target;
    switch (this.targetEnum) {
      case HUB:
        this.targetPose = GenericConstants.HUB_POSE3D;
        break;
      case ALLIANCEZONELEFT:
        this.targetPose = GenericConstants.LEFTALLIANCE;
        break;
      case ALLIANCEZONECENTER:
        this.targetPose = GenericConstants.CENTERALLIANCE;
        break;
      case ALLIANCEZONERIGHT:
        this.targetPose = GenericConstants.RIGHTALLIANCE;
        break;
    }
  }

  public double getVelocity() {
    return topShooterMotorRight.getVelocity().getValueAsDouble() * 60;
  }
  @Override
  public void stopShooter() {
    topShooterMotorRight.stopMotor();
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterVelocityRPM = topShooterMotorRight.getVelocity().getValueAsDouble() * 60;
    inputs.topRightMotorIsConnected = topShooterMotorRight.isConnected();
    inputs.topRightMotorCurrent = topShooterMotorRight.getStatorCurrent().getValueAsDouble();
    inputs.topRightMotorVoltage = topShooterMotorRight.getMotorVoltage().getValueAsDouble();
    inputs.topLeftMotorIsConnected = topShooterMotorLeft.isConnected();
    inputs.topLeftMotorCurrent = topShooterMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.topLeftMotorVoltage = topShooterMotorLeft.getMotorVoltage().getValueAsDouble();
    inputs.bottomRightMotorIsConnected = bottomShooterMotorRight.isConnected();
    inputs.bottomRightMotorCurrent = bottomShooterMotorRight.getStatorCurrent().getValueAsDouble();
    inputs.bottomRightMotorVoltage = bottomShooterMotorRight.getMotorVoltage().getValueAsDouble();
    inputs.bottomLeftMotorIsConnected = bottomShooterMotorLeft.isConnected();
    inputs.bottomLeftMotorCurrent = bottomShooterMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.bottomLeftMotorVoltage = bottomShooterMotorLeft.getMotorVoltage().getValueAsDouble();
  }
}
