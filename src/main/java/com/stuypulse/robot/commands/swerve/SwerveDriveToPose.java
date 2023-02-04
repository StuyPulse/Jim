package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.*;
import com.stuypulse.robot.subsystems.odometry.IOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase{
    private final SwerveDrive swerve;
    private final Pose2d targetPose;
    private final Controller xPID;
    private final Controller yPID;
    private final AnglePIDController anglePID;

    private final BStream aligned;
    
    public SwerveDriveToPose(Pose2d targetPose){
        this.swerve = SwerveDrive.getInstance();
        this.targetPose = targetPose;

        xPID = new PIDController(Translation.P,Translation.I,Translation.D).setSetpointFilter(new MotionProfile(3, 2));
        yPID = new PIDController(Translation.P, Translation.I, Translation.D).setSetpointFilter(new MotionProfile(3, 2));
        anglePID = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        addRequirements(swerve);
    }

    private boolean isAligned() {
        return Math.abs(xPID.getError()) < Alignment.ALIGNED_THRESHOLD_X.get()
            && Math.abs(yPID.getError()) < Alignment.ALIGNED_THRESHOLD_Y.get()
            && Math.abs(anglePID.getError().toDegrees()) < Alignment.ALIGNED_THRESHOLD_ANGLE.get();
    }

    @Override
    public void execute() {

        Pose2d currentState = IOdometry.getInstance().getPose();
    
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            xPID.update(targetPose.getX(), currentState.getX()),
            yPID.update(targetPose.getY(), currentState.getY()),
            anglePID.update( Angle.fromRotation2d(targetPose.getRotation()), Angle.fromRotation2d(currentState.getRotation()))
        );

        swerve.setChassisSpeeds(chassisSpeeds, true);
    }

    @Override
    public boolean isFinished() {
        return aligned.get();
    }
}
