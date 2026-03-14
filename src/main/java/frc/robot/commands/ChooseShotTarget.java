// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.targeting.ShootTargetIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChooseShotTarget extends Command {

  final Translation3d target;
  final ShootTargetIO m_shootTarget;
  final boolean targetAvailable;

  /** Creates a new ChooseShotTarget. */
  public ChooseShotTarget(
      ShootTargetIO shootTarget, Translation3d target, boolean targetAvailable) {
    this.m_shootTarget = shootTarget;
    this.target = target;
    this.targetAvailable = targetAvailable;

    addRequirements(shootTarget);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shootTarget.setTarget(target, targetAvailable);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootTarget.resetTarget();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
