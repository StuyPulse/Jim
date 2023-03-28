package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.swerve.SwerveDriveToScorePose;
import com.stuypulse.robot.subsystems.Manager;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotAlignThenScore extends SequentialCommandGroup {    
    public RobotAlignThenScore() {
        addCommands(
            new SwerveDriveToScorePose(),
            // new ConditionalCommand(
            //     new DoNothingCommand(),
                new RobotScore().withTimeout(1),
                // () -> Manager.getInstance().getGamePiece().isCube()),
            new RobotRelease()
        );
    }
}
