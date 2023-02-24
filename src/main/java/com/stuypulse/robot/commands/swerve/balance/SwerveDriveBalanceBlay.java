package com.stuypulse.robot.commands.swerve.balance;

import com.stuypulse.robot.commands.swerve.SwerveDrivePointWheels;
import com.stuypulse.robot.constants.Settings.AutoBalance;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBalanceBlay extends CommandBase {

    private Number maxSpeed;
    
    Number kK_u = IStream.create(() -> AutoBalance.MAX_SPEED.get() / AutoBalance.MAX_TILT.doubleValue()).number();  // from Zieger-Nichols tuning method
    Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method
    Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * AutoBalance.kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method

    private Number ANGLE_THRESHOLD;

    private Controller control;

    private SwerveDrive swerve;
    private Odometry odometry;

    private double balanceAngle;

    public SwerveDriveBalanceBlay() {
        maxSpeed = AutoBalance.MAX_SPEED;

        ANGLE_THRESHOLD = AutoBalance.ANGLE_THRESHOLD.doubleValue();

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        control = new PIDController(kP, 0, kD).setOutputFilter(x -> -x);

        balanceAngle = 0;
    }

    // private SmartBoolean enabled = new SmartBoolean("Auto Balance/Enabled", false);

    @Override
    public void execute() {
        balanceAngle = swerve.getBalanceAngle().getDegrees();
        control.update(0, balanceAngle);
        
        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            control.getOutput(), 0, 0, odometry.getRotation()));

        SmartDashboard.putNumber("Auto Balance/Speed", control.getOutput());
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
        this.maxSpeed = maxSpeed;

        ANGLE_THRESHOLD = angleTolerance;

        return this;
    }
}