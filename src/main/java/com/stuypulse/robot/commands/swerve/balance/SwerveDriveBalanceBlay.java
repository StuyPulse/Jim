package com.stuypulse.robot.commands.swerve.balance;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.commands.swerve.SwerveDrivePointWheels;
import com.stuypulse.robot.constants.Settings.AutoBalance;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBalanceBlay extends CommandBase {

    private Number maxSpeed;
    
    private Number kK_u = IStream.create(() -> maxSpeed.doubleValue() / AutoBalance.MAX_TILT.doubleValue()).number();  // from Zieger-Nichols tuning method
    private Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method
    private Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * AutoBalance.kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method

    private Number angleThreshold;

    private Controller control;

    private SwerveDrive swerve;
    private Odometry odometry;
    private Plant plant;

    public SwerveDriveBalanceBlay() {
        maxSpeed = AutoBalance.MAX_SPEED;

        angleThreshold = AutoBalance.ANGLE_THRESHOLD.doubleValue();

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        plant = Plant.getInstance();
        control = new PIDController(kP, 0, kD).setOutputFilter(x -> -x);

        addRequirements(swerve, plant);
    }

    // private SmartBoolean enabled = new SmartBoolean("Auto Balance/Enabled", false);

    @Override
    public void execute() {
        control.update(0, swerve.getBalanceAngle().getDegrees());
        
        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            control.getOutput(), 0, 0, odometry.getRotation()));

        SmartDashboard.putNumber("Auto Balance/Speed", control.getOutput());
    }

private boolean timedOut = false;

    @Override
    public boolean isFinished() {
        if (Robot.getMatchState() == MatchState.AUTO && Timer.getMatchTime() < 0.1) {
            timedOut = true;
            return true;
        }
        return control.isDone(angleThreshold.doubleValue());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        swerve.setXMode();
        plant.engage();

        if (timedOut) {
            // LEDController.getInstance().setColor(LEDColor.YELLOW);
        }
    }

    public SwerveDriveBalanceBlay withMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        return this;
    }

    public SwerveDriveBalanceBlay withAngleThreshold(double degreesThreshold) {
        this.angleThreshold = degreesThreshold;
        return this;
    }
}