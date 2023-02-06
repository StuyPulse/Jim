package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase{
    private final SwerveDrive swerve;
    private final Pose2d targetPose;

    // Holonomic control
    private final Controller xController;
    private final Controller yController;
    private final AngleController angleController;
    
    public SwerveDriveToPose(Pose2d targetPose){
        this.swerve = SwerveDrive.getInstance();
        this.targetPose = targetPose;

        xController = new PIDController(Translation.P,Translation.I,Translation.D)
            .setSetpointFilter(new MotionProfile(3, 2));
        yController = new PIDController(Translation.P, Translation.I, Translation.D)
            .setSetpointFilter(new MotionProfile(3, 2));
        angleController =new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
            .setSetpointFilter(new AMotionProfile(5, 4));

        addRequirements(swerve);
    }

    @Override
    public void execute() {

        Pose2d currentState = Odometry.getInstance().getPose();
    
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            xController.update(targetPose.getX(), currentState.getX()),
            yController.update(targetPose.getY(), currentState.getY()),
            angleController.update(Angle.fromRotation2d(targetPose.getRotation()), Angle.fromRotation2d(currentState.getRotation()))
        );

        swerve.setChassisSpeeds(chassisSpeeds, true);
    }
}
