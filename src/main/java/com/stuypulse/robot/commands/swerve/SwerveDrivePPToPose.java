package com.stuypulse.robot.commands.swerve;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.util.SwerveTrajectory;

public class SwerveDrivePPToPose extends SwerveDriveFollowTrajectory {
    public SwerveDrivePPToPose(SwerveTrajectory trajectory) {
        super(trajectory.getTrajectory());
    }
}
