package com.stuypulse.robot.commands.swerve;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.util.SwerveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class SwerveDrivePPToPose extends SwerveDriveFollowTrajectory {
    public SwerveDrivePPToPose() {
    }

    public void getTrajectory(List<SwerveState> poses) {
        PathPlanner.generatePath();
    }

}
