package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Field.*;

import com.stuypulse.robot.constants.Settings.AutoBalance;
import com.stuypulse.robot.constants.Settings.AutoBalance.*;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBalanceWithDualPID extends CommandBase {

    private interface Constants {
        SmartNumber kT_u = new SmartNumber("Auto Balance/With Dual PID/Tu", 0.2);  // from Zieger-Nichols tuning method
        Number kK_u = IStream.create(() -> MAX_SPEED.doubleValue() / AutoBalance.MAX_TILT.doubleValue()).number();  // from Zieger-Nichols tuning method

        Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method
        SmartNumber kI = new SmartNumber("", 0);
        Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method
    }

    private static Number MAX_SPEED;

    private Number DISTANCE_THRESHOLD;
    private Number ANGLE_THRESHOLD;

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Controller tiltController, translationController;
    private final Controller gyroController;
    
    public SwerveDriveBalanceWithDualPID() {
        MAX_SPEED = AutoBalance.MAX_SPEED.doubleValue();

        DISTANCE_THRESHOLD = AutoBalance.DISTANCE_THRESHOLD.doubleValue();
        ANGLE_THRESHOLD = AutoBalance.ANGLE_THRESHOLD.doubleValue();

        this.swerve = SwerveDrive.getInstance();
        this.odometry = Odometry.getInstance();

        tiltController = new PIDController(Tilt.P, Tilt.I, Tilt.D);
        translationController = new PIDController(Translation.P, Translation.I, Translation.D);

        gyroController = new PIDController(Constants.kP, Constants.kI, Constants.kD);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        odometry.overrideNoise(true);
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
        double balanceAngle = getBalanceAngle().getDegrees();
        double target = Units.inchesToMeters(CHARGING_STATION_CENTER.get());
        double offset = tiltController.update(0, balanceAngle) / Math.cos(Math.toRadians(balanceAngle));

        target += offset;
        double currentPosition = odometry.getTranslation().getX();

        double velocity;

        if (translationController.isDone(DISTANCE_THRESHOLD.doubleValue())) {
            velocity = gyroController.update(0, -1 * balanceAngle);
            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                            new ChassisSpeeds(velocity, 0.0, 0.0), 
                                            odometry.getRotation()));
        } else {
            velocity = translationController.update(target, currentPosition);

            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                            new ChassisSpeeds(velocity, 0.0, 0.0), 
                                            odometry.getRotation()));
            
            SmartDashboard.putNumber("Auto Balance/Balance Angle", balanceAngle);
            SmartDashboard.putNumber("Auto Balance/Velocity", velocity);
            SmartDashboard.putNumber("Auto Balance/Target", target);
        }
    }

    @Override 
    public boolean isFinished() {
        return gyroController.isDone(ANGLE_THRESHOLD.doubleValue());
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.stop();
        odometry.overrideNoise(false);
        
        if (!interrupted)
            Plant.getInstance().engage();
    }

    public Command thenPointWheels() {
        return andThen(new SwerveDrivePointWheels(Rotation2d.fromDegrees(90)));
    }

    public Command withTolerance(double maxSpeed, double distanceTolerance, double angleTolerance) {
        MAX_SPEED = maxSpeed;

        ANGLE_THRESHOLD = angleTolerance;
        DISTANCE_THRESHOLD = distanceTolerance;

        return this;
    }
}