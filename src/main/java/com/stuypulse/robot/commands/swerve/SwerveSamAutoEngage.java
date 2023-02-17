package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.Pitch;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveSamAutoEngage extends CommandBase {

    private final SwerveDrive swerve;
    private final Odometry odometry;

    private final Controller tiltController, velocityController;
    
    public SwerveSamAutoEngage() {
        this.swerve = SwerveDrive.getInstance();
        this.odometry = Odometry.getInstance();

        tiltController = new PIDController(6.0/15.0, 0,0);
        velocityController = new PIDController(1,0,0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        odometry.overrideNoise(true);
    }

    @Override
    public void execute() {
        var target = Units.inchesToMeters(Field.CHARGING_STATION_CENTER.getX());
        var balanceAngle = Pitch.calculate(swerve.getGyroPitch(), swerve.getGyroRoll(), swerve.getGyroAngle()).getDegrees();
        var offset = tiltController.update(0, balanceAngle);

        target += offset;
        var currentPosition = odometry.getTranslation().getX();
        var velocity = velocityController.update(target, currentPosition);

        swerve.setChassisSpeeds(new ChassisSpeeds(velocity, 0.0, 0.0));
        
        SmartDashboard.putNumber("Auto Engage/Balance Angle", balanceAngle);
        SmartDashboard.putNumber("Auto Engage/Velocity", velocity);
        SmartDashboard.putNumber("Auto Engage/Target", target);
    }

    @Override 
    public boolean isFinished() {
        return velocityController.isDone(0.1);
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());
        odometry.overrideNoise(false);
    }
}