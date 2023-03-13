package com.stuypulse.robot.util;

import java.util.Optional;

import static com.stuypulse.robot.constants.Settings.Alignment.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveState extends PathPoint  {
 
    private final Pose2d pose;

    private Optional<PathConstraints> constraints;

    public SwerveState(Pose2d targetPose) {
        constraints = Optional.of(new PathConstraints(
            MAX_SPEED.get(),
            MAX_ACCELERATION.get()
        ));

        pose = targetPose;
    }

    public SwerveState(Number x, Number y, Number omega) {
        this(new Pose2d(x.doubleValue(), y.doubleValue(), Rotation2d.fromDegrees(omega.doubleValue())));
    }

    public Pose2d getCurrentPose() {
        return Odometry.getInstance().getPose();
    }

    public Pose2d getTargetPose() {
        return pose;
    }

    // Will mutate arm state
    public SwerveState setConstraints(PathConstraints constraints) {
        this.constraints = Optional.of(constraints);
        return this;
    }

    public SwerveState setConstraints(Number MAX_SPEED, Number MAX_ACCELERATION) {
        this.constraints = Optional.of(new PathConstraints(MAX_SPEED.doubleValue(), MAX_ACCELERATION.doubleValue()));
        return this;
    }

    public double getMaxSpeed() {
        return constraints.get().maxVelocity;
    }

    public double getMaxAcceleration() {
        return constraints.get().maxAcceleration;
    }

}
