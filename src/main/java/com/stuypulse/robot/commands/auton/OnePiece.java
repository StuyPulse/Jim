package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePiece extends SequentialCommandGroup {

    public OnePiece() {
        addCommands(
            new FollowTrajectory(
                PathPlanner.loadPath("1 Piece + Mobility", Motion.CONSTRAINTS)
            ).robotRelative()
        );
    }

}