package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Field.*;

import com.stuypulse.robot.constants.Settings.AutoEngage.*;
import com.stuypulse.robot.constants.Settings.AutoEngage;
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

public class SwerveDriveBalanceWithPlant extends CommandBase {
    public interface Gyro {
        SmartNumber kMaxTilt = new SmartNumber("Auto Engage/Max Tilt (deg)", 15.0); 
        SmartNumber kMaxEngageSpeed = new SmartNumber("Auto Engage/Max Engage Speed (m per s)", 0.65);

        SmartNumber kT_u = new SmartNumber("Auto Engage/Tu", 0.2);  // from Zieger-Nichols tuning method
        Number kK_u = IStream.create(() -> kMaxEngageSpeed.get() / kMaxTilt.get()).number();  // from Zieger-Nichols tuning method

        Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method
        SmartNumber kI = new SmartNumber("", 0);
        Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method
    }

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private Controller controller;
    private Controller gyroController;

    private double balanceAngle;

    public SwerveDriveBalanceWithPlant() {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        controller = new PIDController(Translation.P, Translation.I, Translation.D);
        gyroController = new PIDController(Gyro.kP, Gyro.kI, Gyro.kD);

        balanceAngle = 0;

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

        balanceAngle = getBalanceAngle().getDegrees();

        double target = Units.inchesToMeters(CHARGING_STATION_CENTER.get());
        double currentPosition = odometry.getPose().getX();
        double velocity;

        if (controller.isDone(AutoEngage.DISTANCE_THRESHOLD.get())) {
            velocity = gyroController.update(0, -1 * balanceAngle);
            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                            new ChassisSpeeds(velocity, 0.0, 0.0), 
                                            odometry.getRotation()));
        } else {
            velocity = controller.update(target, currentPosition);
            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                        new ChassisSpeeds(velocity, 0.0, 0.0), 
                                        odometry.getRotation()));
        }

        SmartDashboard.putNumber("Auto Balance/Balance Angle", balanceAngle);
        SmartDashboard.putNumber("Auto Balance/Velocity", velocity);
        SmartDashboard.putNumber("Auto Balance/Target", target);
    }

    @Override
    public boolean isFinished() {
        return gyroController.isDone(AutoEngage.ANGLE_THRESHOLD.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        odometry.overrideNoise(false);

        Plant.getInstance().engage();
    }

    public Command pointWheels() {
        return new SwerveDriveBalanceWithPlant().andThen(new SwerveDrivePointWheels(Rotation2d.fromDegrees(90)));
    }
    
    public Command withTolerance(double maxSpeed, double distanceTolerance, double angleTolerance) {
        Gyro.kMaxEngageSpeed.set(maxSpeed);

        AutoEngage.ANGLE_THRESHOLD.set(distanceTolerance);
        AutoEngage.DISTANCE_THRESHOLD.set(distanceTolerance);

        return new SwerveDriveBalanceWithDualPID();
    }
}
