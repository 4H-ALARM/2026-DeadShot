// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private static final String SHOT_TUNING_KEY = "Shooter/ShotTuning/";

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
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterKp", ShooterConstants.shooterkp);
  private final LoggedTunableNumber shooterki =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterKi", ShooterConstants.shooterki);
  private final LoggedTunableNumber shooterkd =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterKd", ShooterConstants.shooterkd);
  private final LoggedTunableNumber shooterka =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterKa", ShooterConstants.shooterka);
  private final LoggedTunableNumber shooterks =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterKs", ShooterConstants.shooterks);
  private final LoggedTunableNumber shooterkv =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterKv", ShooterConstants.shooterkv);
  private final LoggedTunableNumber shooterMaxAccel =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterMaxAccel", ShooterConstants.shooterMaxAccel);
  private final LoggedTunableNumber shooterMaxSpeed =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterMaxSpeed", ShooterConstants.shooterMaxSpeed);
  private final LoggedTunableNumber shooterJerk =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterJerk", ShooterConstants.shooterJerk);
  private final LoggedTunableNumber shooterFF =
      new LoggedTunableNumber(SHOT_TUNING_KEY + "ShooterFF", ShooterConstants.shooterFF);


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


    shooterVelocityVoltage = new VelocityTorqueCurrentFOC(0).withSlot(0);
    shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.Slot0 = shooterConfig;
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
        shooterVelocityVoltage.withVelocity(speed).withFeedForward(shooterFF.get()));
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
    inputs.topLeftMotorIsConnected = topShooterMotorLeft.isConnected();
    inputs.bottomRightMotorIsConnected = bottomShooterMotorRight.isConnected();
    inputs.bottomLeftMotorIsConnected = bottomShooterMotorLeft.isConnected();
  }
}
