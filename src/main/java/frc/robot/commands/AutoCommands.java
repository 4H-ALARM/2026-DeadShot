// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endeffector.Shooter;
import frc.robot.lib.BLine.*;
import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class AutoCommands {

    public static Command rightSwipe1(FollowPath.Builder builder, double waitinit, double waitend) {
        Path rightSwipe = new Path("RightInitSwipe");
        return Commands.sequence(Commands.waitSeconds(waitinit), builder.build(rightSwipe), Commands.waitSeconds(waitend));
    }
    
    public static Command rightSwipe2(FollowPath.Builder builder, double waitinit, double waitend) {
        Path rightSwipe = new Path("RightSecondSwipe");
        return Commands.sequence(Commands.waitSeconds(waitinit), builder.build(rightSwipe), Commands.waitSeconds(waitend));
    }


    public static Command Right2Swipe(FollowPath.Builder builder, Shooter shooter) {
        return Commands.sequence(rightSwipe1(builder, 0, 0), rightSwipe2(builder, 0, 0));
    }


 
}
