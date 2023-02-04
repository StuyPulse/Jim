package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.IOdometry;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.network.SmartString;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BasicGyroEngage extends CommandBase {
        
    private Controller control;

    private SmartNumber MAX_TILT, MAX_ENGAGE_SPEED;

    public BasicGyroEngage() {

        MAX_TILT = new SmartNumber("Auto Engage/Max Tilt (deg)", 15.0);
        MAX_ENGAGE_SPEED = new SmartNumber("Auto Engage/Max Engage Speed (m per s)", 0.25);

        control = new PIDController(IStream.create(() -> MAX_ENGAGE_SPEED.get() / MAX_TILT.get()).number(), 0, 0);
    }

    @Override
    public void execute() {
        double speed = control.update(
            0,
            -1 * Pitch.calculate(IOdometry.getInstance().getGyroPitch(), swerve.getGyroRoll(), swerve.getAngle()).getDegrees());
        
        swerve.setStates(new ChassisSpeeds(speed, 0, 0), true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setStates(new ChassisSpeeds(), true);
    }

}