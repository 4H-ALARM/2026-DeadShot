// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Constants;

/** Add your docs here. */
public class IntakeConstants {

  public static final double rotationGearRatio = 3 * 5;
  public static final double rotationUpDegrees = 0;
  public static final double rotationDownDegrees = 88;

  // Motor IDs
  public static final int rotationMotorID = 30;
  public static final int rotationMotorFollowID = 17;
  public static final int intakingMotorID = 2;
  public static final int intakingFollowMotorID = 3;
  public static final String canbus = "endEffector";
  // PID tuning for the intake
  public static final double angleMotorkp = 150;
  public static final double angleMotorki = 0.01;
  public static final double angleMotorkd = 0.1;
  public static final double angleMotorks = 0.2;
  public static final double angleMotorkv = 1.87;
  public static final double angleMotorkg = 0.5;
  public static final double angleMotorka = 0.25;
  public static final double angleMotionMagicCruiseVelocityRotationsPerSecond = 0.75;
  public static final double angleMotionMagicAccelerationDegreesPerSecondSquared = 1000*5;
  public static final double angleMotionMagicJerkDegreesPerSecondCubed = 0.0;
  public static final double anglePositionToleranceDegrees = 3.0;
  public static final double jostleIntakeVelocity = 0.05;

  public static final double intakingMotorkp = 1.5;
  public static final double intakingMotorki = 0.0;
  public static final double intakingMotorkd = 0.0;
  public static final double intakingMotorks = 0.01;
  public static final double intakingMotorkv = 0.0;
  public static final double intakingMotorka = 0.0;
}
