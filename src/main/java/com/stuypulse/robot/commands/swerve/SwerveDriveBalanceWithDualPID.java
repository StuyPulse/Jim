package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Field.*;
import static com.stuypulse.robot.constants.Settings.AutoBalance.*;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBalanceWithDualPID extends CommandBase {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Controller tiltController, translationController;
    
    public SwerveDriveBalanceWithDualPID() {
        this.swerve = SwerveDrive.getInstance();
        this.odometry = Odometry.getInstance();

        tiltController = new PIDController(Tilt.P, Tilt.I, Tilt.D);
        translationController = new PIDController(Translation.P, Translation.I, Translation.D);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        odometry.overrideNoise(true);
    }

    private Rotation2d getBalanceAngle() {
        Rotation2d pitch = swerve.getGyroPitch();
        Rotation2d roll = swerve.getGyroRoll();
        Rotation2d yaw = odometry.getRotation();
        
        double facingSlope = pitch.getTan() * yaw.getCos() + roll.getTan() * yaw.getSin();
        double maxSlope = Math.sqrt(Math.pow(roll.getTan(), 2) + Math.pow(pitch.getTan(), 2));

        return Rotation2d.fromRadians(Math.signum(facingSlope) * Math.atan(maxSlope));
    }

    @Override
    public void execute() {
        double balanceAngle = getBalanceAngle().getDegrees();
        double target = Units.inchesToMeters(CHARGING_STATION_CENTER.get());
        double offset = tiltController.update(0, balanceAngle) / Math.cos(Math.toRadians(balanceAngle));

        target += offset;
        double currentPosition = odometry.getTranslation().getX();
        double velocity = translationController.update(target, currentPosition);

        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                        new ChassisSpeeds(velocity, 0.0, 0.0), 
                                        odometry.getRotation()));
        
        SmartDashboard.putNumber("Auto Engage/Balance Angle", balanceAngle);
        SmartDashboard.putNumber("Auto Engage/Velocity", velocity);
        SmartDashboard.putNumber("Auto Engage/Target", target);
    }

    // @Override 
    // public boolean isFinished() {
    //     return translationController.isDone(0.1);
    // }

    @Override 
    public void end(boolean interrupted) {
        // swerve.setChassisSpeeds(new ChassisSpeeds());
        odometry.overrideNoise(false);
    }
}