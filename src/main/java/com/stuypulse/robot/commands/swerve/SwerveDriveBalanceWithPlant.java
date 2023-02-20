package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Field.*;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBalanceWithPlant extends CommandBase {

    public interface Translation {
        SmartNumber P = new SmartNumber("Auto Balance/Balance With Plant/Translation/kP", 1);
        SmartNumber I = new SmartNumber("Auto Balance/Balance With Plant/Translation/kI", 0);
        SmartNumber D = new SmartNumber("Auto Balance/Balance With Plant/Translation/kD", 0);

        SmartNumber DISTANCE_THRESHOLD = new SmartNumber("Auto Balance/Distance Threshold", 0.05);
    }

    public interface Tilt {
        SmartNumber ANGLE_THRESHOLD = new SmartNumber("Auto Balance/Angle Threshold", 2);

        SmartNumber kT_u = new SmartNumber("Auto Engage/Tu", 0.2);  // from Zieger-Nichols tuning method
        Number kK_u = IStream.create(() -> kMaxEngageSpeed.get() / ANGLE_THRESHOLD.get()).number();  // from Zieger-Nichols tuning method
        
        Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method

        Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method
    }

    private static SmartNumber kMaxEngageSpeed = new SmartNumber("Auto Engage/Max Engage Speed (m per s)", 0.1);

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private Controller controller;
    private Controller gyroController;

    private double balanceAngle;

    public SwerveDriveBalanceWithPlant() {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();

        controller = new PIDController(Translation.P, Translation.I, Translation.D);
        gyroController = new PIDController(Tilt.kP, 0, Tilt.kD);

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

        if (controller.isDone(Translation.DISTANCE_THRESHOLD.get())) {
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
        return gyroController.isDone(5);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        odometry.overrideNoise(false);

        Plant.getInstance().engage();
        var positions = swerve.getModulePositions();
        for (SwerveModulePosition position : positions) {
            position.angle = Rotation2d.fromDegrees(90).plus(position.angle);
        }
    }
}
