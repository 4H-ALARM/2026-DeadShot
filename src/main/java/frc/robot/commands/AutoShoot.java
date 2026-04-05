// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endeffector.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.targeting.ShootTargetIO;

import java.util.function.DoubleSupplier;

public class AutoShoot {
  private AutoShoot() {}

  /**
   * Returns a command that spins the shooter at the lookup-table RPM and aims the drive at the
   * current shoot target. The driver retains translational control via joysticks.
   */
  public static Command autoShootWithXLock(
      Shooter shooter,
      Drive drive,
      Intake intake,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Trigger xlockTrigger) {

        return Commands.parallel(
            getHoodAngleCommand(shooter),
            // Spin shooter at lookup RPM
            getShootCommand(shooter, intake),

            // sequence to work xlock or aim at target, depending on whether the xlock trigger is active or not
            Commands.repeatingSequence(
                RunWithButton.runWhileButtonReleased(xlockTrigger,
                    getDriveCommand(xSupplier, ySupplier, drive, shooter.getShootTarget())),
                RunWithButton.runWhileButtonPressed(xlockTrigger,
                    getXLockCommand(drive))
            )
        ).finallyDo(() -> shooter.stopShooter());
  }

  public static Command autoShoot(
    Shooter shooter,
    Drive drive,
    Intake intake,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier) {
      return Commands.parallel(
            getHoodAngleCommand(shooter),
            // Spin shooter at lookup RPM
            getShootCommand(shooter, intake),

            getDriveCommand(xSupplier, ySupplier, drive, shooter.getShootTarget())
        ).finallyDo(() -> shooter.stopShooter());
  }

  private static Command getDriveCommand(
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    Drive drive,
    ShootTargetIO shootTarget) {
    return DriveCommands.joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            () -> {
              Translation2d robotXY = drive.getPose().getTranslation();
              Translation2d targetXY = shootTarget.getTarget().toTranslation2d();
              Translation2d toTarget = targetXY.minus(robotXY);
              return new Rotation2d(toTarget.getX(), toTarget.getY()).rotateBy(new Rotation2d(Math.PI));
            });
  }

  private static Command getXLockCommand(Drive drive) {
    // command to xlock the drive, this is a startRun instead of an instant command,
    // because we don't want the command to complete until cancelled.
    return Commands.startRun(drive::stopWithX, () -> {}, drive);
  }

  private static Command getHoodAngleCommand(Shooter shooter) {
    return Commands.run(() -> shooter.setHoodAngle(shooter.getActiveTargetHoodAngle()));
  }

  private static Command getShootCommand(Shooter shooter, Intake intake) {
    return new ShootBall(shooter, intake, -5900.0);
  }
}
