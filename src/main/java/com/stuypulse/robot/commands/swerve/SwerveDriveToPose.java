package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.AlignmentCommand.Rotation;
import com.stuypulse.robot.constants.Settings.AlignmentCommand.Translation;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveToPose extends CommandBase{
    SwerveDrive swerve;
    Pose2d targetPose;
    PIDController xPID;
    PIDController yPID;
    AnglePIDController anglePID;
    
    public SwerveDriveToPose(SwerveDrive swerve, Pose2d targetPose){
        this.swerve = swerve;
        this.targetPose = targetPose;

        xPID = new PIDController(Translation.P,Translation.I,Translation.D);
        yPID = new PIDController(Translation.P, Translation.I, Translation.D);
        anglePID = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D);

        addRequirements(swerve);
    }
    @Override
    public void execute() {

        Pose2d currentState = Odometry.getInstance().getPose();
    
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            xPID.update(targetPose.getX(), currentState.getX()),
            yPID.update(targetPose.getY(), currentState.getY()),
            anglePID.update( Angle.fromRotation2d(targetPose.getRotation()), Angle.fromRotation2d(currentState.getRotation()))
        );

        swerve.setChassisSpeeds(chassisSpeeds, true);
    }
}
