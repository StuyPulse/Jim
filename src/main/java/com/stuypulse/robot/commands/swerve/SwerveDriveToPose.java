package com.stuypulse.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase {
    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Supplier<Pose2d> references;
    private Pose2d target;

    private Timer timer;

    private TrapezoidProfile xProfile;
    private PIDController xFeedback;
    
    private TrapezoidProfile yProfile;
    private PIDController yFeedback;

    private TrapezoidProfile angleProfile;
    private AnglePIDController angleFeedback;

    public SwerveDriveToPose(Supplier<Pose2d> references) {
        this.references = references;

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        timer = new Timer();

        xFeedback = new PIDController(Alignment.Translation.P, 0, Alignment.Translation.D);
        yFeedback = new PIDController(Alignment.Translation.P, 0, Alignment.Translation.D);
        angleFeedback = new AnglePIDController(Alignment.Rotation.P, 0, Alignment.Rotation.D);

        addRequirements(swerve);// odometry
    }

    public SwerveDriveToPose(Pose2d reference) {
        this(() -> reference);
    }

    public void initialize() {
        target = references.get();
        var pose = odometry.getTranslation();
        var speeds = swerve.getChassisSpeeds();

        timer.restart();

        // Create translational profiles
        var constraints = new Constraints(Alignment.MAX_VELOCITY.get(), Alignment.MAX_ACCELERATION.get());

        xProfile = new TrapezoidProfile(
            constraints, 
            new State(target.getX(), 0), 
            new State(pose.getX(), speeds.vxMetersPerSecond));

        yProfile = new TrapezoidProfile(
            constraints, 
            new State(target.getY(), 0), 
            new State(pose.getY(), speeds.vyMetersPerSecond));

        // Create angle profile
        var angleConstraints = new Constraints(Alignment.MAX_ANGULAR_VELOCITY.get(), Alignment.MAX_ANGULAR_ACCELERATION.get());

        var measurement = odometry.getRotation();
        double goalMinDistance = target.getRotation().minus(measurement).getRadians();
        
        angleProfile = new TrapezoidProfile(
            angleConstraints, 
            new State(goalMinDistance + measurement.getRadians(), 0),
            new State(measurement.getRadians(), speeds.omegaRadiansPerSecond));

    }

    public void execute() {
        Pose2d pose = odometry.getPose();

        final double time = timer.get();
        var xState = xProfile.calculate(time);
        var yState = yProfile.calculate(time);
        var angleState = angleProfile.calculate(time);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xState.velocity + xFeedback.update(xState.position, pose.getX()),
            yState.velocity + yFeedback.update(yState.position, pose.getY()),
            angleState.velocity,
            pose.getRotation()
        );

        swerve.setChassisSpeeds(speeds);
    }
}
