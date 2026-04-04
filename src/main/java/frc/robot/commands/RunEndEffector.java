// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.IntakeConstants;
import frc.robot.subsystems.endeffector.Shooter;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunEndEffector extends Command {
  private static final double INTAKE_HALF_RETRACT_DEGREES =
      MathUtil.interpolate(
          IntakeConstants.rotationDownDegrees, IntakeConstants.rotationUpDegrees, 0.4);
  private static final double INTAKE_QUARTER_RETRACT_DEGREES =
      MathUtil.interpolate(
          IntakeConstants.rotationDownDegrees, IntakeConstants.rotationUpDegrees, 0.2);
  private static final double INTAKE_DWELL_SECONDS = 0.2;

  private final double m_indexerSpeed;
  private final Shooter m_shooter;
  private final Intake m_intake;
  private final Timer m_phaseTimer = new Timer();
  private IntakeMotionState m_intakeMotionState = IntakeMotionState.MOVING_TO_HALF;

  private enum IntakeMotionState {
    MOVING_TO_HALF,
    HOLDING_AT_HALF,
    MOVING_TO_QUARTER,
    HOLDING_AT_QUARTER,
    STAY_DOWN
  }

  /** Creates a new RunEndEffector. */
  public RunEndEffector(Shooter shooter, Intake intake, double indexerSpeed) {
    this.m_shooter = shooter;
    this.m_intake = intake;
    this.m_indexerSpeed = indexerSpeed;
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_intake.shouldJostleOnShoot()) {
      m_intakeMotionState = IntakeMotionState.MOVING_TO_HALF;
      commandIntakeAngle(INTAKE_HALF_RETRACT_DEGREES);
    } else {
      m_intakeMotionState = IntakeMotionState.STAY_DOWN;
      m_intake.setRotationDown();
    }
    m_phaseTimer.restart();
    m_shooter.spinShooterFromLookup();
    m_shooter.stopIndexer();
    m_intake.setIntakeSpeed(-5900 / 60);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spinShooterFromLookup();
    updateIntakeMotion();

    if (m_shooter.isShooterAtTargetVelocity()) {
      m_shooter.setIndexerSpeed(m_indexerSpeed);
    } else {
      m_shooter.setIndexerSpeed(m_indexerSpeed / 6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_phaseTimer.stop();
    m_shooter.stopShooter();
    m_shooter.stopIndexer();
    m_intake.stopIntake();
    m_intake.setRotationDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void updateIntakeMotion() {
    switch (m_intakeMotionState) {
      case MOVING_TO_HALF:
        if (m_intake.isAtAngle(
            INTAKE_HALF_RETRACT_DEGREES, IntakeConstants.anglePositionToleranceDegrees)) {
          m_intakeMotionState = IntakeMotionState.HOLDING_AT_HALF;
          m_phaseTimer.restart();
        }
        break;
      case HOLDING_AT_HALF:
        if (m_phaseTimer.hasElapsed(INTAKE_DWELL_SECONDS)) {
          m_intakeMotionState = IntakeMotionState.MOVING_TO_QUARTER;
          commandIntakeAngle(INTAKE_QUARTER_RETRACT_DEGREES);
        }
        break;
      case MOVING_TO_QUARTER:
        if (m_intake.isAtAngle(
            INTAKE_QUARTER_RETRACT_DEGREES, IntakeConstants.anglePositionToleranceDegrees)) {
          m_intakeMotionState = IntakeMotionState.HOLDING_AT_QUARTER;
          m_phaseTimer.restart();
        }
        break;
      case HOLDING_AT_QUARTER:
        if (m_phaseTimer.hasElapsed(INTAKE_DWELL_SECONDS)) {
          m_intakeMotionState = IntakeMotionState.MOVING_TO_HALF;
          commandIntakeAngle(INTAKE_HALF_RETRACT_DEGREES);
        }
        break;
      case STAY_DOWN:
        break;
    }
  }

  private void commandIntakeAngle(double targetAngleDegrees) {
    m_intake.setAngle(targetAngleDegrees);
  }
}
