// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Constants;

/** Add your docs here. */
public class IntakeConstants {

  public static final double rotationGearRatio = 3 * 5;
  public static final double rotationUpDegrees = 2;
  public static final double rotationDownDegrees = 98;

  // Motor IDs
  public static final int rotationMotorID = 30;
  public static final int rotationMotorFollowID = 17;
  public static final int intakingMotorID = 2;
  // public static final int intakingMotorFollowID = 2;
  public static final String canbus = "endEffector";
  // PID tuning for the intake
  public static final double angleMotorkp = 22.5;
  public static final double angleMotorki = 0.01;
  public static final double angleMotorkd = 0.25;
  public static final double angleMotorks = 0;
  public static final double angleMotorkv = 1.47;
  public static final double angleMotorkg = 0.72;
  public static final double angleMotorka = 0.18;
}
