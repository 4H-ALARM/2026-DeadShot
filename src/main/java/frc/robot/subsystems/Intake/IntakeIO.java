// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public boolean rotationMotorConnected;
    public boolean rotationMotorFollowerConnected;
    public boolean intakingMotorConnected;

    public double rotationDegrees;
    public double rotationSpeedDegreesPerSecond;
    public double rotationSetpointDegrees;
  }

  public void resetEncoder();

  public void changeAngleTest(double speed);

  public void setAngle(double angleDegrees);

  public default double getAngle() {
    return 0;
  }

  public void setIntakeSpeed(double speed);

  public void stopIntake();

  public default boolean isIntakeUp() {
    return true;
  };

  public default void updateTuningValues() {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
