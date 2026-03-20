// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunEndEffector extends Command {
  double m_shooterSpeed;
  double m_indexerSpeed;
  Shooter m_shooter;
  /** Creates a new RunEndEffector. */
  public RunEndEffector(Shooter shooter, double indexerSpeed) {
    this.m_shooter = shooter;
    this.m_shooterSpeed = shooter.getLookupRpm();
    this.m_indexerSpeed = indexerSpeed;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.spinShooterFromLookup();
    m_shooter.setIndexerSpeed(m_indexerSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_shooter.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
