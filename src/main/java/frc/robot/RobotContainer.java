// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Constants.GenericConstants;
import frc.lib.Constants.ShooterConstants;
import frc.lib.Constants.SwerveConstants;
import frc.lib.catalyst.hardware.MotorType;
import frc.lib.catalyst.mechanisms.RotationalMechanism;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SelectTarget;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.endeffector.IndexerIOKraken;
import frc.robot.subsystems.endeffector.PhaseshiftIO;
import frc.robot.subsystems.endeffector.Shooter;
import frc.robot.subsystems.endeffector.ShooterIOKraken;
import frc.robot.subsystems.targeting.ShootTargetIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.ThrottledRotationalMechanism;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoChooser;

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
  private final Vision vision;
  private final ShootTargetIO shootTarget = new ShootTargetIO(GenericConstants.HUB_POSE3D, true);
  private SwerveDriveKinematics swerveKinematics;

  private final CommandXboxController PilotController = new CommandXboxController(0);
  private final CommandXboxController OperatorController = new CommandXboxController(1);

  private final DeployIntake deployIntake;
//   private final DeployIntake deployIntakeAuto;
  private final Command driveDefaultCommand;
  private final Command indexerReverseCommand;
  private final Command autoShootCommand;
  private final Command intakeCommand;
  private final Command ejectCommand;
  private final Command intakeCommandAuto;
  private final Command resetGyroCommand;
  private final Command ShootCommand;
  private final Command ShootFromTowerCommand;


  private final LoggedDashboardChooser<Command> autoChooser;

  private final DoubleSupplier pilotForwardInput = () -> -PilotController.getLeftY();
  private final DoubleSupplier pilotStrafeInput = () -> -PilotController.getLeftX();
  private final DoubleSupplier pilotRotateInput = () -> -PilotController.getRightX();

  private final Trigger pilotRightBumper = PilotController.rightBumper();
  private final Trigger pilotRightTrigger = PilotController.rightTrigger();
  private final Trigger pilotLeftTrigger = PilotController.leftTrigger();
  private final Trigger pilotLeftBumper = PilotController.leftBumper();
  private final Trigger pilotB = PilotController.b();

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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, backLeft),
                new VisionIOPhotonVision(camera1Name, backRight),
                new VisionIOPhotonVision(camera2Name, sideLeft),
                new VisionIOPhotonVision(camera3Name, sideRight)
                );

        shooter =
            new Shooter(
                new ShooterIOKraken(),
                drive,
                new IndexerIOKraken(),
                new PhaseshiftIO(),
                shootTarget,
                PilotController);

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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, backLeft, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, backRight, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, sideLeft, drive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, sideRight, drive::getPose));
        shooter =
            new Shooter(
                new ShooterIOKraken(),
                drive,
                new IndexerIOKraken(),
                new PhaseshiftIO(),
                shootTarget,
                PilotController);

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
        shooter =
            new Shooter(
                new ShooterIOKraken(),
                drive,
                new IndexerIOKraken(),
                new PhaseshiftIO(),
                shootTarget,
                PilotController);

        intake = new Intake(new IntakeIOKraken());

        vision = new Vision(drive::addVisionMeasurement//,
            // new VisionIO() {},
            // new VisionIO() {}
            );
        break;
    }

    FollowPath.Builder pathBuilder = new FollowPath.Builder(
            drive,                      // The drive subsystem to require
            drive::getPose,             // Supplier for current robot pose
            drive::getChassisSpeeds,    // Supplier for current speeds
            drive::runVelocity,               // Consumer to drive the robot
            new PIDController(5.0, 0.0, 0.0),    // Translation PID
            new PIDController(3.0, 0.0, 0.0),    // Rotation PID
            new PIDController(2.0, 0.0, 0.0)     // Cross-track PID
            ).withDefaultShouldFlip()                // Auto-flip for red alliance 
            .withPoseReset(drive::setPose);  // Reset odometry at path start

    deployIntake = new DeployIntake(intake);
    // deployIntakeAuto = new DeployIntake(intake);
    driveDefaultCommand =
        DriveCommands.joystickDrive(
            drive, pilotForwardInput, pilotStrafeInput, pilotRotateInput);
    indexerReverseCommand =
        Commands.runEnd(() -> shooter.setIndexerSpeed(-6300 / 60), () -> shooter.setIndexerSpeed(0));
    autoShootCommand = AutoShoot.autoShoot(shooter, drive, intake, pilotForwardInput, pilotStrafeInput).withTimeout(4);
    ShootCommand = AutoShoot.autoShoot(shooter, drive, intake, pilotForwardInput, pilotStrafeInput);
    intakeCommand =
        Commands.runEnd(() -> intake.setIntakeSpeed(-5900 / 60), () -> intake.setIntakeSpeed(0), intake);
    ejectCommand =
        Commands.runEnd(() -> intake.setIntakeSpeed(5900 / 60), () -> intake.setIntakeSpeed(0), intake);
    intakeCommandAuto =
        Commands.runEnd(() -> intake.setIntakeSpeed(-5900 / 60), () -> intake.setIntakeSpeed(0), intake);
    resetGyroCommand =
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
            .ignoringDisable(true);
    ShootFromTowerCommand =
        Commands.runEnd(() -> shooter.spinShooter(1825 / 60), () -> shooter.stopShooter(), shooter);
    NamedCommands.registerCommand("Shoot", autoShootCommand);
    NamedCommands.registerCommand("Deploy intake", new DeployIntake(intake));
    NamedCommands.registerCommand("Intake", intakeCommandAuto);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
    drive.setDefaultCommand(driveDefaultCommand);

    // PilotController.rightBumper().onTrue(shooter.getHood().goTo(-10));
    // PilotController.leftBumper().onTrue(shooter.getHood().goTo(0));
    // PilotController.a().onTrue(shooter.getHood().goTo(-25));
    // PilotController.leftBumper().whileTrue(new InstantCommand(() -> shooter.setHoodAngle(-10)));

    // intake.setDefaultCommand(
    //     new InstantCommand(() -> intake.changeAngleTest(controller.getLeftY()), intake));

    // Lock to 0° when A button is held

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller.y().onTrue(new SimulateShotTrajectory(drive, shooter));
    // PilotController.leftBumper().onTrue(new InstantCommand(() -> shooter.spinShooter(1)));
    // PilotController.a()
    //     .whileTrue(new InstantCommand(() -> shooter.spinShooter(-.9)))
    //     .whileTrue(new InstantCommand(() -> shooter.setIndexerSpeed(-1)))
    //     .whileTrue(new InstantCommand(() -> intake.setIntakeSpeed(.8)))
    //     .whileFalse(new InstantCommand(() -> shooter.setIndexerSpeed(0)))
    //     .whileFalse(new InstantCommand(() -> shooter.spinShooter(0)))
    //     .whileFalse(new InstantCommand(() -> intake.setIntakeSpeed(0)));
    // PilotController.leftTrigger()
    //     .whileTrue(
    //         Commands.runEnd(() -> shooter.spinShooter(1750 / 60), () -> shooter.stopShooter()));

    pilotRightBumper
        .whileTrue(
            Commands.runEnd(() -> shooter.setIndexerSpeed(-5900 / 60), () -> shooter.setIndexerSpeed(0)));
    pilotRightTrigger
        .whileTrue(
            ShootCommand).onFalse(new InstantCommand(() -> shooter.stopShooter()) );
    pilotLeftTrigger
        .toggleOnTrue(
            intakeCommand);
    pilotLeftBumper
        .onTrue(deployIntake);

    // TODO: this requires the shooter, but would not allow indexer to run from the pilot command.
    OperatorController.rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> shooter.spinShooter(2500/60), () -> shooter.stopShooter())
        );
    OperatorController.b()
        .onTrue( new InstantCommand(() -> shooter.setTarget(GenericConstants.RIGHTPASSING)))
        .onFalse(new InstantCommand(() -> shooter.resetTarget()));

    OperatorController.x()
        .onTrue(new InstantCommand( () -> shooter.setTarget(GenericConstants.LEFTPASSING)))
        .onFalse(new InstantCommand(() -> shooter.resetTarget()));

    OperatorController.povLeft()
        .onTrue(new InstantCommand( () -> intake.toggleIntakeJostling()));
    // OperatorController.rightTrigger()
    //     .whileTrue(
    //         ShootFromTowerCommand
    //     );
    // OperatorController.leftBumper()
    //     .whileTrue(
    //         ejectCommand
    //     );

    // Reset gyro to 0° when B button is pressed
    // pilotB
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));

    //     OperatorController.b().onTrue(Commands.runOnce(() -> shootTarget.resetTarget(),
    // shootTarget));

    //     OperatorController.leftTrigger()
    //         .and(OperatorController.a().negate())
    //         .onTrue(new ChooseShotTarget(shootTarget, GenericConstants.LEFTALLIANCE, false));

    //     OperatorController.rightTrigger()
    //         .and(OperatorController.a().negate())
    //         .onTrue(new ChooseShotTarget(shootTarget, GenericConstants.RIGHTALLIANCE, false));

    //     OperatorController.leftBumper()
    //         .and(OperatorController.a().negate())
    //         .onTrue(new ChooseShotTarget(shootTarget, GenericConstants.CENTERALLIANCE, false));

    //     OperatorController.leftTrigger()
    //         .and(OperatorController.a())
    //         .onTrue(new ChooseShotTarget(shootTarget, GenericConstants.LEFTNEUTRAL, false));

    //     OperatorController.rightTrigger()
    //         .and(OperatorController.a())
    //         .onTrue(new ChooseShotTarget(shootTarget, GenericConstants.RIGHTNEUTRAL, false));

    //     OperatorController.leftBumper()
    //         .and(OperatorController.a())
    //         .onTrue(new ChooseShotTarget(shootTarget, GenericConstants.MIDDLENEUTRAL, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
