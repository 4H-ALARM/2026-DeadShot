// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    boolean rotationMotorConnected;
    boolean rotationMotorFollowerConnected;
    boolean intakingMotorConnected;

    double rotationDegrees;
    double rotationSpeed;
  }

  public void resetEncoder();

  public void changeAngleTest(double speed);

  public void setAngle(double angleDegrees);

  public void setIntakeSpeed(double speed);

  public void stopIntake();

  public default void updateInputs(IntakeIOInputs inputs) {}
}
