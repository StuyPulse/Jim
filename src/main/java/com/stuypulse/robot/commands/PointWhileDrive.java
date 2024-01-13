package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.odometry.OdometryImpl;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

// we will be aiming with the swerve!
// use camera to get the distance to goal for RPM 
// for direction: 
/*
 * 1) get the differential; where will we be when we shoot the ball using a feed-forward model
 * 2) 
 * https://www.chiefdelphi.com/t/high-refresh-rate-vision-processing/387949/32
 * https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472/2
 */
public class PointWhileDrive extends CommandBase {
    
    private SwerveDrive swerve;
    private Translation2d target;
    private Rotation2d targetAngle;
    private Rotation2d currentAngle;
    private AnglePIDController angleController;

    private Odometry odometry = OdometryImpl.getInstance();



    public PointWhileDrive(SwerveDrive swerve, Translation2d target){
        this.swerve = swerve;
        this.target = target;
        angleController = new AnglePIDController(1, 1, 1);


        addRequirements(swerve);
    }

    

    @Override
    public void execute(){
        currentAngle = odometry.getPose().getRotation().plus(swerve.getGyroAngle());
        
        
        //calculate targetAngle
        targetAngle = new Rotation2d(
            odometry.getPose().getX() - target.getX(),
            odometry.getPose().getY() - target.getY())
            .plus(Rotation2d.fromRadians(Math.PI));
        
        swerve.aimAt(targetAngle);
        
        

        swerve.drive(
            new Vector2D(swerve.getVelocity()),
            angleController.update(
                Angle.fromRotation2d(targetAngle), 
                Angle.fromRotation2d(currentAngle)));
        
        SmartDashboard.putNumber("Swerve/targetAngle", targetAngle.getDegrees());
        //System.out.println(targetAngle);
        // System.out.println(new Rotation2d(
        //     swerve.getPose().getX() - target.getX(),
        //     swerve.getPose().getY() - target.getY()));

        System.out.println(odometry.getPose());
    }
}
