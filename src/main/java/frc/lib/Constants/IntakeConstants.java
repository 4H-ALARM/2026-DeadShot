// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Constants;

/** Add your docs here. */
public class IntakeConstants {

  public static final double rotationGearRatio = 60.0 / 20.0;

  // Motor IDs
  public static final int rotationMotorID = 30;
  public static final int rotationMotorFollowID = 17;
  public static final int intakingMotorID = 2;
  public static final int intakingMotorFollowID = 2;
  public static final String canbus = "endEffector";
  // PID tuning for the intake
  public static final double angleMotorkp = 1;
  public static final double angleMotorki = 0;
  public static final double angleMotorkd = 0;
}
