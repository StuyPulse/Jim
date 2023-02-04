package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlignToPose extends SequentialCommandGroup{
    
    public AlignToPose(Pose2d targetPose, RobotContainer robot) {
        addCommands(
            new SwerveDriveToPose(new Pose2d(robot.odometry.getPose().getX(), targetPose.getY(), robot.odometry.getRotation())),
            new SwerveDriveToPose(new Pose2d(targetPose.getX(), robot.odometry.getPose().getY(), robot.odometry.getRotation())),
            new SwerveDriveToPose(targetPose)
            );
    }
}
