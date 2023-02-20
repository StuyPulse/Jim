package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBlayBalance extends CommandBase {

    private static SmartNumber kMaxTilt = new SmartNumber("Auto Engage/Max Tilt (deg)", 15.0); 
    private static SmartNumber kMaxEngageSpeed = new SmartNumber("Auto Engage/Max Engage Speed (m per s)", 0.65);

    private static SmartNumber kT_u = new SmartNumber("Auto Engage/Tu", 0.2);  // from Zieger-Nichols tuning method
    private static Number kK_u = IStream.create(() -> kMaxEngageSpeed.get() / kMaxTilt.get()).number();  // from Zieger-Nichols tuning method

    private static Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method
    private static SmartNumber kI = new SmartNumber("", 0);
    private static Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method

    private Controller control;

    private SwerveDrive swerve;
    private Odometry odometry;

    private double balanceAngle;

    public SwerveDriveBlayBalance() {

        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        control = new PIDController(kP, kI, kD);

        balanceAngle = 0;
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
        balanceAngle = -1 * getBalanceAngle().getDegrees();
        var speed = control.update(
            0,
            balanceAngle);
        
        swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
            speed, 0, 0, odometry.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return balanceAngle > -8 && balanceAngle < 8;
    }

    @Override
    public void end(boolean interrupted) {

        swerve.stop();
        
        // Plant.getInstance().engage()'[]\';
        // var positions = swerve.getModulePositions();
        // for (SwerveModulePosition position : positions) {
        //     position.angle = Rotation2d.fromDegrees(90).plus(position.angle);
        // }
    }
}