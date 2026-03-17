// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.lib.Constants.ShooterConstants;

/** Add your docs here. */
public class IndexerIOKraken implements IndexerIO {

  TalonFX m_indexerMotor;
  TalonFX m_indexerMotorFollower;
  TalonFXConfiguration m_configurator;
  Slot0Configs m_Configs;
  VelocityTorqueCurrentFOC m_targetVelocity;

  public IndexerIOKraken() {
    m_indexerMotor = new TalonFX(ShooterConstants.indexerMotorID, ShooterConstants.shooterCanbus);
    m_indexerMotorFollower =
        new TalonFX(ShooterConstants.indexerMotorFollowerID, ShooterConstants.shooterCanbus);
    m_indexerMotorFollower.setControl(
        new Follower(ShooterConstants.indexerMotorID, MotorAlignmentValue.Opposed));
    m_Configs = new Slot0Configs()
    .withKP(ShooterConstants.indexerkp)
    .withKI(ShooterConstants.indexerki)
    .withKD(ShooterConstants.indexerkd)
    .withKS(ShooterConstants.indexerks)
    .withKV(ShooterConstants.indexerkv)
    .withKA(ShooterConstants.indexerka);
    m_configurator = new TalonFXConfiguration();
    m_configurator.Slot0 = m_Configs;
    m_indexerMotor.getConfigurator().apply(m_configurator);
    m_targetVelocity = new VelocityTorqueCurrentFOC(0);

  }
  ///
  public void setIndexerSpeed(double speed) {
    //m_indexerMotor.set(speed);
    m_indexerMotor.setControl(m_targetVelocity.withVelocity(speed));
  }

  public void stopIndexer() {

    m_indexerMotor.set(0);
  }

  public void updateInputs(IndexerIOInputs inputs) {

    // need to figure out how many encoder units equals one full rotation
  }
}
