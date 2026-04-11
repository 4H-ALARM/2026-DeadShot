// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endeffector.Shooter;
import frc.robot.subsystems.intake.Intake;

import java.util.function.DoubleSupplier;

public class AutoShoot {
  private AutoShoot() {}

  /**
   * Returns a command that spins the shooter at the lookup-table RPM and aims the drive at the
   * current shoot target. The driver retains translational control via joysticks.
   */
  public static Command autoShoot(
      Shooter shooter,
      Drive drive,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return Commands.parallel(
        // Spin shooter at lookup RPM
        new ShootBall(shooter, intake, -5900.0),
        //Commands.run(() -> shooter.setHoodAngle(shooter.getActiveTargetHoodAngle())),
        // Aim drive at target
        DriveCommands.joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            () -> {
              Translation2d robotXY = drive.getPose().getTranslation();
              Translation2d targetXY =
                  new Translation2d(
                      shooter.getShootTarget().getTarget().getX(),
                      shooter.getShootTarget().getTarget().getY());
              Translation2d toTarget = targetXY.minus(robotXY);
              return new Rotation2d(toTarget.getX(), toTarget.getY()).rotateBy(new Rotation2d(Math.PI));
            }))
        .finallyDo(() -> shooter.stopShooter());
  }
}
