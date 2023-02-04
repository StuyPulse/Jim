package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.AlignmentCommand.Rotation;
import com.stuypulse.robot.constants.Settings.AlignmentCommand.Translation;
import com.stuypulse.robot.subsystems.odometry.IOdometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase{
    private final SwerveDrive swerve;
    private final Pose2d targetPose;
    private final PIDController xPID;
    private final PIDController yPID;
    private final AnglePIDController anglePID;
    
    public SwerveDriveToPose(Pose2d targetPose){
        this.swerve = SwerveDrive.getInstance();
        this.targetPose = targetPose;

        xPID =  (PIDController) new PIDController(Translation.P,Translation.I,Translation.D)
            .setSetpointFilter(new MotionProfile(3, 2));
        yPID = (PIDController) new PIDController(Translation.P, Translation.I, Translation.D)
            .setSetpointFilter(new MotionProfile(3, 2));
        anglePID = (AnglePIDController )new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
            .setSetpointFilter(new AMotionProfile(5, 4));

        addRequirements(swerve);
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
}
