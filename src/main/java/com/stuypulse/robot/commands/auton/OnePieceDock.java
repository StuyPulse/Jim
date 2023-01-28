package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceDock extends SequentialCommandGroup {

    public OnePieceDock() {
        addCommands(
            new FollowTrajectory(
                PathPlanner.loadPath("1 Piece + Dock", Motion.CONSTRAINTS)
            ).robotRelative()
            // new BasicGyroEngage(robot.swerve)   
        );
    }

}