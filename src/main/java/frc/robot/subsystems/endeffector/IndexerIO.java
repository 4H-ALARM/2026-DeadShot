// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {
  @AutoLog
  public class IndexerIOInputs {
    double indexerVelocityRPM;
    boolean isIndexerMotorConnected;
    boolean isFollowerMotorConnected;
  }

  public default void setIndexerSpeed(double indexerSpeed) {}

  public default void stopIndexer() {}

  public default void updateInputs(IndexerIOInputs inputs) {}
}
