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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Constants.GenericConstants;
import frc.lib.Constants.ShooterConstants;
import frc.lib.enums.TargetEnum;
import frc.lib.util.LoggedTunableNumber;

/** Add your docs here. */
public class ShooterIOKraken implements ShooterIO {
  private TalonFX topShooterMotorRight;
  private TalonFX topShooterMotorLeft;
  private TalonFX bottomShooterMotorRight;
  private TalonFX bottomShooterMotorLeft;
  private TalonFX hoodMotor;
  private TalonFX hoodMotorFollower;

  private CANcoder hoodEncoder;

  private Slot0Configs shooterConfig;
  private MotionMagicConfigs shooterMotionMagicConfig;
  private Slot0Configs hoodConfig;
  private MotionMagicConfigs hoodMotionMagicConfig;
  private TalonFXConfiguration shooterMotorConfig;
  private TalonFXConfiguration hoodMotorConfig;

  private TargetEnum targetEnum;
  private Pose2d robotPose;
  private Pose2d turretPose;
  private Translation3d targetPose;

  private PositionVoltage hoodPositionVoltage;
  private VelocityTorqueCurrentFOC shooterVelocityVoltage;

  private final LoggedTunableNumber shooterkp =
      new LoggedTunableNumber("Shooter/kp", ShooterConstants.shooterkp);
  private final LoggedTunableNumber shooterki =
      new LoggedTunableNumber("Shooter/ki", ShooterConstants.shooterki);
  private final LoggedTunableNumber shooterkd =
      new LoggedTunableNumber("Shooter/kd", ShooterConstants.shooterkd);
  private final LoggedTunableNumber shooterka =
      new LoggedTunableNumber("Shooter/ka", ShooterConstants.shooterka);
  private final LoggedTunableNumber shooterks =
      new LoggedTunableNumber("Shooter/ks", ShooterConstants.shooterks);
  private final LoggedTunableNumber shooterkv =
      new LoggedTunableNumber("Shooter/kv", ShooterConstants.shooterkv);
  private final LoggedTunableNumber shooterMaxAccel =
      new LoggedTunableNumber("Shooter/maxAccel", ShooterConstants.shooterMaxAccel);
  private final LoggedTunableNumber shooterMaxSpeed =
      new LoggedTunableNumber("Shooter/maxSpeed", ShooterConstants.shooterMaxSpeed);
  private final LoggedTunableNumber shooterJerk =
      new LoggedTunableNumber("Shooter/jerk", ShooterConstants.shooterJerk);
  private final LoggedTunableNumber hoodToMotorRatio =
      new LoggedTunableNumber("Hood/hoodToMotorRatio", ShooterConstants.hoodToMotorRatio);

  private final LoggedTunableNumber hoodkp =
      new LoggedTunableNumber("Hood/kp", ShooterConstants.hoodkp);
  private final LoggedTunableNumber hoodki =
      new LoggedTunableNumber("Hood/ki", ShooterConstants.hoodki);
  private final LoggedTunableNumber hoodkd =
      new LoggedTunableNumber("Hood/kd", ShooterConstants.hoodkd);
  private final LoggedTunableNumber hoodka =
      new LoggedTunableNumber("Hood/ka", ShooterConstants.hoodka);
  private final LoggedTunableNumber hoodks =
      new LoggedTunableNumber("Hood/ks", ShooterConstants.hoodks);
  private final LoggedTunableNumber hoodkv =
      new LoggedTunableNumber("Hood/kv", ShooterConstants.hoodkv);
  private final LoggedTunableNumber hoodkg =
      new LoggedTunableNumber("Hood/kg", ShooterConstants.hoodkg);
  private final LoggedTunableNumber hoodMaxAccel =
      new LoggedTunableNumber("Hood/maxAccel", ShooterConstants.hoodMaxAccel);
  private final LoggedTunableNumber hoodMaxSpeed =
      new LoggedTunableNumber("Hood/maxSpeed", ShooterConstants.hoodMaxSpeed);
  private final LoggedTunableNumber hoodJerk =
      new LoggedTunableNumber("Hood/jerk", ShooterConstants.hoodJerk);

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
    hoodMotor = new TalonFX(ShooterConstants.hoodMotorID, ShooterConstants.shooterCanbus);
    hoodMotorFollower =
        new TalonFX(ShooterConstants.hoodMotorFollowerID, ShooterConstants.shooterCanbus);
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

    hoodConfig =
        new Slot0Configs()
            .withKP(hoodkp.get())
            .withKI(hoodki.get())
            .withKD(hoodkd.get())
            .withKA(hoodka.get())
            .withKS(hoodks.get())
            .withKV(hoodkv.get())
            .withKG(hoodkg.get());

    hoodMotionMagicConfig =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(hoodMaxAccel.get())
            .withMotionMagicCruiseVelocity(hoodMaxSpeed.get())
            .withMotionMagicJerk(hoodJerk.get());

    shooterVelocityVoltage = new VelocityTorqueCurrentFOC(0).withSlot(0);
    shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.Slot0 = shooterConfig;
    // shooterMotorConfig.MotionMagic = shooterMotionMagicConfig;

    hoodMotorConfig = new TalonFXConfiguration();
    hoodMotorConfig.Slot0 = hoodConfig;
    hoodMotorConfig.MotionMagic = hoodMotionMagicConfig;
    hoodMotorConfig.Feedback.FeedbackRemoteSensorID = ShooterConstants.hoodEncoderID;
    hoodMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    hoodMotorConfig.Feedback.SensorToMechanismRatio = hoodToMotorRatio.get();

    topShooterMotorRight.getConfigurator().apply(shooterMotorConfig);
    hoodMotor.getConfigurator().apply(hoodMotorConfig);
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

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          hoodConfig =
              new Slot0Configs()
                  .withKP(hoodkp.get())
                  .withKI(hoodki.get())
                  .withKD(hoodkd.get())
                  .withKA(hoodka.get())
                  .withKS(hoodks.get())
                  .withKV(hoodkv.get())
                  .withKG(hoodkg.get());
          hoodMotor.getConfigurator().apply(hoodConfig);
        },
        hoodkp,
        hoodki,
        hoodkd,
        hoodka,
        hoodks,
        hoodkv,
        hoodkg);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          hoodMotionMagicConfig =
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(hoodMaxAccel.get())
                  .withMotionMagicCruiseVelocity(hoodMaxSpeed.get())
                  .withMotionMagicJerk(hoodJerk.get());
          hoodMotor.getConfigurator().apply(hoodMotionMagicConfig);
        },
        hoodMaxAccel,
        hoodMaxSpeed,
        hoodJerk);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          hoodMotorConfig.Feedback.SensorToMechanismRatio = hoodToMotorRatio.get();
          hoodMotor.getConfigurator().apply(hoodMotorConfig);
        },
        hoodToMotorRatio);
  }

  @Override
  public void setShooterSpeed(double speed) {
    // topShooterMotorRight.set(speed);
    topShooterMotorRight.setControl(shooterVelocityVoltage.withVelocity(speed));
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

  @Override
  public void setHoodAngle(double angleDegrees) {
    // still need to find the angle before this will work;

    double motorRotations = (angleDegrees) * ShooterConstants.hoodGearRatio / 360;

    hoodMotor.setControl(hoodPositionVoltage.withPosition(motorRotations));
  }

  public void updateInputs(ShooterIOInputs inputs) {
    SmartDashboard.putNumber("Speed", topShooterMotorRight.getVelocity().getValueAsDouble() * 60);
  }
}
