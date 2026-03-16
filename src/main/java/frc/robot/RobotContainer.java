// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Constants.SwerveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.endeffector.IndexerIOKraken;
import frc.robot.subsystems.endeffector.Shooter;
import frc.robot.subsystems.endeffector.ShooterIOKraken;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //   private final Vision vision;
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private SwerveDriveKinematics swerveKinematics;

  private final CommandXboxController controller = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(SwerveConstants.FrontLeft),
                new ModuleIOTalonFX(SwerveConstants.FrontRight),
                new ModuleIOTalonFX(SwerveConstants.BackLeft),
                new ModuleIOTalonFX(SwerveConstants.BackRight));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(camera0Name, robotToCamera0),
        //         new VisionIOPhotonVision(camera1Name, robotToCamera1));

        shooter = new Shooter(new ShooterIOKraken(), drive, new IndexerIOKraken());

        intake = new Intake(new IntakeIOKraken());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(SwerveConstants.FrontLeft),
                new ModuleIOSim(SwerveConstants.FrontRight),
                new ModuleIOSim(SwerveConstants.BackLeft),
                new ModuleIOSim(SwerveConstants.BackRight));

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        shooter = new Shooter(new ShooterIOKraken(), drive, new IndexerIOKraken());

        intake = new Intake(new IntakeIOKraken());
        break;

      default:
        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter(new ShooterIOKraken(), drive, new IndexerIOKraken());

        intake = new Intake(new IntakeIOKraken());
        break;
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // intake.setDefaultCommand(
    //     new InstantCommand(() -> intake.changeAngleTest(controller.getLeftY()), intake));

    // Lock to 0° when A button is held

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller.y().onTrue(new SimulateShotTrajectory(drive, shooter));
    controller
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                () -> shooter.spinShooter(controller.getLeftY() * 0.25),
                () -> shooter.spinShooter(0),
                shooter,
                drive));

    controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> intake.setIntakeSpeed(controller.getLeftY() * 0.25),
                () -> intake.setIntakeSpeed(0),
                intake,
                drive));

    controller
        .x()
        .whileTrue(
            Commands.runEnd(
                () -> shooter.setIndexerSpeed(controller.getLeftY() * 0.25),
                () -> shooter.setIndexerSpeed(0),
                drive));

    controller
        .a()
        .whileTrue(
            Commands.runEnd(
                () -> intake.changeAngleTest(controller.getLeftY() * 0.5),
                () -> intake.changeAngleTest(0),
                intake,
                drive));

    controller
        .b()
        .whileTrue(
            Commands.runEnd(
                () -> shooter.spinHood(controller.getLeftY() * 0.25),
                () -> shooter.spinHood(0),
                shooter,
                drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
