/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Driver.Drive;
import com.stuypulse.robot.constants.Settings.Driver.Turn;
import com.stuypulse.robot.constants.Settings.Driver.Turn.GyroFeedback;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.lang.annotation.Target;
import java.util.Optional;

public class SwerveDriveAndPoint extends CommandBase {

    private final SwerveDrive swerve;
    private final Plant plant;
    private final Odometry odometry;

    private VStream speed;
    private IStream turn;

    private final Gamepad driver;
    
    private Translation2d target;
    

    private final AngleController gyroFeedback;

    public SwerveDriveAndPoint(Gamepad driver, Translation2d target) {
        this.driver = driver;
        this.target = target;

        swerve = SwerveDrive.getInstance();
        plant = Plant.getInstance();
        odometry = Odometry.getInstance();

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Drive.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Drive.POWER.get()),
                x -> x.mul(Drive.MAX_TELEOP_SPEED.get()),
                new VRateLimit(Drive.MAX_TELEOP_ACCEL),
                new VLowPassFilter(Drive.RC)
            );


        gyroFeedback = new AnglePIDController(GyroFeedback.P, GyroFeedback.I, GyroFeedback.D);
        //is this different from a general anglePID controller?

        addRequirements(swerve, plant);
    }

    @Override
    public void initialize() {
    }

    private boolean isTurnInDeadband() {
        return Math.abs(turn.get()) < Turn.DEADBAND.get();
    }

    private boolean isDriveInDeadband() {
        return driver.getLeftStick().magnitude() < Drive.DEADBAND.get();
    }


    private Rotation2d targetAngle;
    private Rotation2d currentAngle;
    @Override
    public void execute() {
        
        
        currentAngle = odometry.getPose().getRotation();

        targetAngle = new Rotation2d(
            target.getX() - odometry.getPose().getX(),
            target.getY() - odometry.getPose().getY()
        );

        swerve.aimAt(targetAngle);
        
        

        swerve.drive(
            new Vector2D(swerve.getVelocity()),
            gyroFeedback.update( // not sure if it is right, or I should make a new PID
                Angle.fromRotation2d(targetAngle), 
                Angle.fromRotation2d(currentAngle)));
        


        // unplant if driving in endgame
        if (Timer.getMatchTime() < 30) {
            if ((driver.getLeftStick().magnitude() > 0.5) ||
                (driver.getRightStick().magnitude() > 0.5)) {

                plant.disengage();
            }
        }

        // if planted then X
        if (plant.isEngaged() || driver.getRawStartButton() || driver.getRawSelectButton()) {
            //swerve.setXMode();
            
            new PointWhileDrive(swerve, new Translation2d());
        }

        // if in robot relative mode, set raw chassis speed
        else if (driver.getRawLeftBumper()) {
            swerve.drive(new Vector2D(speed.get().x, speed.get().y), 
            gyroFeedback.update(
                Angle.fromRotation2d(targetAngle), 
                Angle.fromRotation2d(currentAngle)));
            //swerve.setChassisSpeeds(new ChassisSpeeds(speed.get().y, -speed.get().x, angularVel));
            //swerve.setChassisSpeeds(new ChassisSpeeds(speed.y, -speed.x, -angularVel));
        }

        
        // use the angularVelocity for drive otherwise
        else {
            swerve.drive(speed.get(), 
            gyroFeedback.update(
                Angle.fromRotation2d(targetAngle), 
                Angle.fromRotation2d(currentAngle)));
            //swerve.drive(speed, angularVel);
        }
    }

}
