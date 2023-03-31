package com.stuypulse.robot.commands.swerve.balance;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Settings.AutoBalance;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveVelocityBalance extends CommandBase {

    private SmartBoolean timedOut = new SmartBoolean("Auto Balance/Timed Out", false);

    private Number speed;
    private Number angleVelocityThreshold;

    private double angleVelocity;
    private double previousAngle;

    private SwerveDrive swerve;
    private Odometry odometry;
    private Plant plant;

    public SwerveDriveVelocityBalance() {
        speed = AutoBalance.MAX_SPEED;
        angleVelocityThreshold = AutoBalance.ANGULAR_VELOCITY_THRESHOLD.doubleValue();

        angleVelocity = 0;
        previousAngle = 0;

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        plant = Plant.getInstance();

        addRequirements(swerve, plant);
    }

    // private SmartBoolean enabled = new SmartBoolean("Auto Balance/Enabled", false);

    @Override
    public void execute() {
        double currentAngle = swerve.getBalanceAngle().getDegrees();
        angleVelocity = (currentAngle - previousAngle) / 0.02;
        previousAngle = currentAngle;

        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            currentAngle < 0 ? -speed.doubleValue() : speed.doubleValue(), 0, 0, odometry.getRotation()));

        SmartDashboard.putNumber("Auto Balance/CS Angular Velocity (deg per s)", angleVelocity);
    }

    @Override
    public boolean isFinished() {
        if (Robot.getMatchState() == MatchState.AUTO && Timer.getMatchTime() < 0.1) {
            timedOut.set(true);;
            return true;
        }
        return angleVelocity > angleVelocityThreshold.doubleValue();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        swerve.setXMode();
        plant.engage();

        if (timedOut.get()) {
            LEDController.getInstance().setColor(LEDColor.YELLOW, 1);
        }
    }

    public SwerveDriveVelocityBalance withMaxSpeed(double speed) {
        this.speed = speed;
        return this;
    }

    public SwerveDriveVelocityBalance withAngleThreshold(double angleVelocityThreshold) {
        this.angleVelocityThreshold = angleVelocityThreshold;
        return this;
    }
}