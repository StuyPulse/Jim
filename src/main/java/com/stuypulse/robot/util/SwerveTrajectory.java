package com.stuypulse.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.stuypulse.robot.constants.Settings.Alignment.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveTrajectory {

    // public static PathPoint generateBarrierAvoidanceTrajectory(Pose2d start, Pose2d end) {
    //     SwerveTrajectory trajectory = new SwerveTrajectory(start, end);
    // }

    private final PathPlannerTrajectory trajectory;
    private final PathConstraints constraints;

    public SwerveTrajectory(PathConstraints constraints, PathPoint... points) {
        this.constraints = constraints;
        trajectory = PathPlanner.generatePath(constraints, Arrays.asList(points));
    }

    public SwerveTrajectory(PathConstraints constraints, Pose2d... poses) {
        this.constraints = constraints;

        List<PathPoint> pathPoint = new ArrayList<>();
        for (int i = 0; i < poses.length; i++) {
            pathPoint.add(new PathPoint(poses[i].getTranslation(), Odometry.getInstance().getRotation(), poses[i].getRotation()));
        }

        trajectory = PathPlanner.generatePath(constraints, pathPoint);
    }

    public SwerveTrajectory(Pose2d... poses) {
        this(new PathConstraints(MAX_SPEED.get(), MAX_ACCELERATION.get()), poses);
    }

    public Pose2d getCurrentPose() {
        return Odometry.getInstance().getPose();
    }

    public PathPlannerTrajectory getTrajectory() {
        return trajectory;
    }

    public double getMaxSpeed() {
        return constraints.maxVelocity;
    }

    public double getMaxAcceleration() {
        return constraints.maxAcceleration;
    }
}
