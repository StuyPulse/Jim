package com.stuypulse.robot.commands.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.SwerveTrajectory;

public class SwerveDriveToScorePose extends SwerveDriveFollowTrajectory {

    private static PathPlannerTrajectory getTrajectory() {
        SwerveTrajectory trajectory = new SwerveTrajectory(Odometry.getInstance().getPose(), 
                                                Manager.getInstance().getScorePose());
        return trajectory.getTrajectory();
    }   

    public SwerveDriveToScorePose() {
        super(getTrajectory());
    }
}
