package com.stuypulse.robot.commands.swerve;

import static com.stuypulse.robot.constants.Field.*;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveBalanceWithDualPID extends CommandBase {

    SmartNumber DISTANCE_THRESHOLD = new SmartNumber("Auto Balance/Distance Threshold", 0.05);

    public interface Translation {
        SmartNumber P = new SmartNumber("Auto Balance/Translation/kP", 0.05);
        SmartNumber I = new SmartNumber("Auto Balance/Translation/kI", 0);
        SmartNumber D = new SmartNumber("Auto Balance/Translation/kD", 0);
    }

    public interface Tilt {
        SmartNumber P = new SmartNumber("Auto Balance/Tilt/kP", 0.05);
        SmartNumber I = new SmartNumber("Auto Balance/Tilt/kI", 0);
        SmartNumber D = new SmartNumber("Auto Balance/Tilt/kD", 0);
    }

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Controller tiltController, translationController;
    private final Controller gyroController;
    
    public SwerveDriveBalanceWithDualPID() {
        this.swerve = SwerveDrive.getInstance();
        this.odometry = Odometry.getInstance();

        tiltController = new PIDController(Tilt.P, Tilt.I, Tilt.D);
        translationController = new PIDController(Translation.P, Translation.I, Translation.D);

        gyroController = new PIDController(Tilt.P, Tilt.I, Tilt.D);

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

        if (translationController.isDone(DISTANCE_THRESHOLD.get())) {
            velocity = gyroController.update(0, -1 * balanceAngle);
            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                            new ChassisSpeeds(velocity, 0.0, 0.0), 
                                            odometry.getRotation()));
        } else {
            velocity = translationController.update(target, currentPosition);

            swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                                            new ChassisSpeeds(velocity, 0.0, 0.0), 
                                            odometry.getRotation()));
            
            SmartDashboard.putNumber("Auto Engage/Balance Angle", balanceAngle);
            SmartDashboard.putNumber("Auto Engage/Velocity", velocity);
            SmartDashboard.putNumber("Auto Engage/Target", target);
        }
    }

    @Override 
    public boolean isFinished() {
        return gyroController.isDone(2);
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