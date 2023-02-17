package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Field.*;
import static com.stuypulse.robot.constants.Settings.AutoBalance.*;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.Pitch;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveSamAutoEngage extends CommandBase {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Controller tiltController, translationController;
    
    public SwerveSamAutoEngage() {
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

    @Override
    public void execute() {
        double target = Units.inchesToMeters(CHARGING_STATION_CENTER.getX());
        double balanceAngle = Pitch.calculate(swerve.getGyroPitch(), swerve.getGyroRoll(), swerve.getGyroAngle()).getDegrees();
        double offset = tiltController.update(0, balanceAngle);

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

    @Override 
    public boolean isFinished() {
        return translationController.isDone(0.1);
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());
        odometry.overrideNoise(false);
    }
}