// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.lib.Constants.ShooterConstants;

public class IndexerIOKraken implements IndexerIO {

  private final TalonFX m_indexerMotor;
  private final TalonFX m_indexerMotorFollower;

  private final TalonFXConfiguration m_configurator;
  private final Slot0Configs m_slot0Configs;

  private final MotionMagicVelocityTorqueCurrentFOC m_motionMagicVelocity;

  public IndexerIOKraken() {
    m_indexerMotor =
        new TalonFX(ShooterConstants.indexerMotorID, ShooterConstants.shooterCanbus);

    m_indexerMotorFollower =
        new TalonFX(ShooterConstants.indexerMotorFollowerID, ShooterConstants.shooterCanbus);

    m_indexerMotorFollower.setControl(
        new Follower(ShooterConstants.indexerMotorID, MotorAlignmentValue.Opposed));

    m_slot0Configs =
        new Slot0Configs()
            .withKP(ShooterConstants.indexerkp)
            .withKI(ShooterConstants.indexerki)
            .withKD(ShooterConstants.indexerkd)
            .withKS(ShooterConstants.indexerks)
            .withKV(ShooterConstants.indexerkv)
            .withKA(ShooterConstants.indexerka);

    m_configurator = new TalonFXConfiguration();
    m_configurator.Slot0 = m_slot0Configs;

    m_configurator.MotionMagic.MotionMagicCruiseVelocity =
        ShooterConstants.indexerCruiseVelocity;
    m_configurator.MotionMagic.MotionMagicAcceleration =
        ShooterConstants.indexerAcceleration;
    m_configurator.MotionMagic.MotionMagicJerk =
        ShooterConstants.indexerJerk;

    m_indexerMotor.getConfigurator().apply(m_configurator);

    m_motionMagicVelocity = new MotionMagicVelocityTorqueCurrentFOC(0);
  }

  public void setIndexerSpeed(double velocityRPS) {
    m_indexerMotor.setControl(
        m_motionMagicVelocity
            .withVelocity(velocityRPS)
            .withFeedForward(7));
  }

  public void stopIndexer() {
    m_indexerMotor.set(0);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocityRPM = m_indexerMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.isIndexerMotorConnected = m_indexerMotor.isConnected();
    inputs.isFollowerMotorConnected = m_indexerMotorFollower.isConnected();
  }
}
