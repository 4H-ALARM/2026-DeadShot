// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Constants.ShooterConstants;
import frc.robot.subsystems.endeffector.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RevShooter extends Command {
  /** Creates a new ShootBall. */
  private double m_speed;

  private Shooter m_shooter;

  public RevShooter(Shooter shooter, double speed) {
    this.m_speed = speed;
    this.m_shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.spinShooter(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.getShooterVelocity() > m_speed - ShooterConstants.shooterRevTolerance
        && m_shooter.getShooterVelocity() < m_speed + ShooterConstants.shooterRevTolerance;
  }
}
