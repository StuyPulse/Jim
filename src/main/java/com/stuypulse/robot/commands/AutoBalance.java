package com.stuypulse.robot.commands;

import static com.stuypulse.robot.constants.Field.*;
import static com.stuypulse.robot.constants.Settings.AutoBalance.*;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.Pitch;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private Controller controller;

    private SmartNumber balanceAngle;

    public AutoBalance() {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        controller = new PIDController(P, I, D);

        balanceAngle = new SmartNumber("Auto Balance/Angle", 0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        odometry.overrideNoise(true);
    }

    @Override
    public void execute() {

        balanceAngle.set(Pitch.calculate(swerve.getGyroPitch(), swerve.getGyroRoll(), swerve.getGyroAngle()).getDegrees());

        double targetX = Units.inchesToMeters(CHARGING_STATION_CENTER.getX());
        double x = odometry.getPose().getX();

        double speed = controller.update(targetX, x);
        swerve.setChassisSpeeds(new ChassisSpeeds(speed, 0, 0));

        SmartDashboard.putNumber("Auto Balance/Speed", speed);
        SmartDashboard.putNumber("Auto Balance/Target", targetX);
        SmartDashboard.putNumber("Auto Balance/Current X", x);
    }

    @Override
    public boolean isFinished() {
        return balanceAngle.get() < ANGLE_THRESHOLD.get() && controller.isDone(DISTANCE_THRESHOLD.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());
    }
}
