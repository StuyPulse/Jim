package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.odometry.OdometryImpl;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.network.SmartString;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BasicGyroEngage extends CommandBase {
        
    private Controller control;

    private SmartNumber MAX_TILT, MAX_ENGAGE_SPEED;

    SwerveDrive swerve;

    public BasicGyroEngage() {

        swerve = SwerveDrive.getInstance();

        MAX_TILT = new SmartNumber("Auto Engage/Max Tilt (deg)", 15.0);
        MAX_ENGAGE_SPEED = new SmartNumber("Auto Engage/Max Engage Speed (m per s)", 0.25);

        control = new PIDController(IStream.create(() -> MAX_ENGAGE_SPEED.get() / MAX_TILT.get()).number(), 0, 0);
    }

    private Rotation2d getBalanceAngle() {

        Rotation2d pitch = swerve.getGyroPitch();
        Rotation2d roll = swerve.getGyroRoll();
        Rotation2d yaw = swerve.getGyroAngle();
        
        double facingSlope = pitch.getTan() * yaw.getCos() + roll.getTan() * yaw.getSin();
        double maxSlope = Math.sqrt(Math.pow(roll.getTan(), 2) + Math.pow(pitch.getTan(), 2));

        return Rotation2d.fromRadians(Math.signum(facingSlope) * Math.atan(maxSlope));
    }

    @Override
    public void execute() {
        double speed = control.update(
            0,
            -1 * getBalanceAngle().getDegrees());
        
        swerve.setChassisSpeeds(new ChassisSpeeds(speed, 0, 0), true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds(), true);
    }

}