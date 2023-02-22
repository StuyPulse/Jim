package com.stuypulse.robot.commands.swerve.balance;

import com.stuypulse.robot.commands.swerve.SwerveDrivePointWheels;
import com.stuypulse.robot.constants.Settings.AutoBalance;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBalanceBlay extends CommandBase {

    private double MAX_SPEED;
    
    Number kK_u = IStream.create(() -> MAX_SPEED / AutoBalance.MAX_TILT.doubleValue()).number();  // from Zieger-Nichols tuning method
    Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method
    Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * AutoBalance.kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method

    private Number ANGLE_THRESHOLD;

    private Controller control;

    private SwerveDrive swerve;
    private Odometry odometry;

    private double balanceAngle;

    public SwerveDriveBalanceBlay() {
        MAX_SPEED = AutoBalance.MAX_SPEED.doubleValue();

        ANGLE_THRESHOLD = AutoBalance.ANGLE_THRESHOLD.doubleValue();

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        control = new PIDController(kP, 0, kD);

        balanceAngle = 0;
    }

    private Rotation2d getBalanceAngle() {
        Rotation2d pitch = swerve.getGyroPitch();
        Rotation2d roll = swerve.getGyroRoll();
        Rotation2d yaw = odometry.getRotation();
        
        double facingSlope = pitch.getTan() * yaw.getCos() + roll.getTan() * yaw.getSin();
        double maxSlope = Math.sqrt(Math.pow(roll.getTan(), 2) + Math.pow(pitch.getTan(), 2));

        SmartDashboard.putNumber("facingSlope", Math.signum(facingSlope));

        return Rotation2d.fromRadians(Math.signum(facingSlope) * Math.atan(maxSlope));
    }

    @Override
    public void execute() {
        balanceAngle = getBalanceAngle().getDegrees();
        var speed = control.update(
            0,
            balanceAngle);
        
        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            speed, 0, 0, odometry.getRotation()));

        SmartDashboard.putNumber("Auto Balance/Balance Angle (deg)", balanceAngle);
        SmartDashboard.putNumber("Auto Balance/Speed", speed);
        SmartDashboard.putNumber("Auto Balance/Odometry Angle", odometry.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        return control.isDone(ANGLE_THRESHOLD.doubleValue());
    }

    @Override
    public void end(boolean interrupted) {

        swerve.stop();
        
        Plant.getInstance().engage();
    }

    public Command thenPointWheels() {
        return andThen(new SwerveDrivePointWheels(Rotation2d.fromDegrees(90)));
    }

    public Command withTolerance(double maxSpeed, double angleTolerance) {
        MAX_SPEED = maxSpeed;

        ANGLE_THRESHOLD = angleTolerance;

        return this;
    }
}