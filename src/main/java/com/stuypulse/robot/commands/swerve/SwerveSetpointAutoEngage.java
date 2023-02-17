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

public class SwerveSetpointAutoEngage extends CommandBase {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private Controller controller;

    private double balanceAngle;

    public SwerveSetpointAutoEngage() {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        controller = new PIDController(Translation.P, Translation.I, Translation.D);

        balanceAngle = 0;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        odometry.overrideNoise(true);
    }

    @Override
    public void execute() {

        balanceAngle = Pitch.calculate(swerve.getGyroPitch(), swerve.getGyroRoll(), swerve.getGyroAngle()).getDegrees();

        double target = Units.inchesToMeters(CHARGING_STATION_CENTER.getX());
        double currentPosition = odometry.getPose().getX();
        double velocity = controller.update(target, currentPosition);
        
        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                        new ChassisSpeeds(velocity, 0.0, 0.0), 
                                        odometry.getRotation()));

        SmartDashboard.putNumber("Auto Balance/Balance Angle", balanceAngle);
        SmartDashboard.putNumber("Auto Balance/Velocity", velocity);
        SmartDashboard.putNumber("Auto Balance/Target", target);
    }

    @Override
    public boolean isFinished() {
        return balanceAngle < ANGLE_THRESHOLD.get() && controller.isDone(DISTANCE_THRESHOLD.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());
        odometry.overrideNoise(false);
    }
}
