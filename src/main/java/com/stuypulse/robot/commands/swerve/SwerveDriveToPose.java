package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.AngleFeedthrough;
import com.stuypulse.robot.util.Feedthrough;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.filters.Derivative;
import com.stuypulse.stuylib.streams.vectors.filters.VMotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase {
    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Supplier<Pose2d> references;

    private Pose2d target;

    private VMotionProfile motionProfile;
    private Controller xController;
    private Controller yController;

    private AngleController angleController;

    public SwerveDriveToPose(Supplier<Pose2d> references) {
        this.references = references;

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        motionProfile = new VMotionProfile(3, 2);
        xController = new Feedthrough().add(new PIDController(0, 0, 0));
        yController = new Feedthrough().add(new PIDController(0, 0, 0));

        angleController = new AngleFeedthrough()
            .add(new AnglePIDController(0, 0, 0))
            .setSetpointFilter(new AMotionProfile(5, 4));

        addRequirements(swerve);// odometry
    }

    public SwerveDriveToPose(Pose2d reference) {
        this(() -> reference);
    }

    public void initialize() {
        target = references.get();
    }

    public void execute() {

        Pose2d pose = odometry.getPose();

        Vector2D profiledTarget = motionProfile.get(new Vector2D(target.getTranslation()));

        xController.update(profiledTarget.x, pose.getX());
        yController.update(profiledTarget.y, pose.getY());
        angleController.update(
            Angle.fromRotation2d(target.getRotation()), 
            Angle.fromRotation2d(pose.getRotation()));

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.getOutput(),
            yController.getOutput(),
            angleController.getOutput(),
            pose.getRotation()
        );

        swerve.setChassisSpeeds(speeds);
    }
}
