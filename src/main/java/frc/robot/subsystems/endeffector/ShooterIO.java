// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.enums.TargetEnum;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {

    // public Rotation2d hoodAngleDegrees;
    public double topLeftVelocityRPM;
    public boolean topLeftMotorIsConnected;
    public double topLeftMotorCurrent;
    public double topLeftMotorVoltage;
    public double topRightVelocityRPM;
    public boolean topRightMotorIsConnected;
    public double topRightMotorCurrent;
    public double topRightMotorVoltage;
    public double bottomLeftVelocityRPM;
    public boolean bottomLeftMotorIsConnected;
    public double bottomLeftMotorCurrent;
    public double bottomLeftMotorVoltage;
    public double bottomRightVelocityRPM;
    public boolean bottomRightMotorIsConnected;
    public double bottomRightMotorCurrent;
    public double bottomRightMotorVoltage;
    // public boolean hoodMotorIsConnected;
  }

  // public default Rotation2d getHoodAngle() {
  //   return new Rotation2d();
  // }
  // ;

  public default void changeTarget(TargetEnum target) {}

  public default void updateTuningValues() {}

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterSpeed(double speed) {}

  public default double getVelocity() {
    return 0;
  }

  // public default void setHoodAngle(double angleDegrees) {}

  public default void stopShooter(){}
}
