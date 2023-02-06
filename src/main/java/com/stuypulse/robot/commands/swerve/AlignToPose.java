package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlignToPose extends SequentialCommandGroup{
    
    public AlignToPose(Pose2d targetPose) {
        addCommands(
            new SwerveDriveToPoseX(targetPose),
            new SwerveDriveToPoseY(targetPose)
        );
    }
}
