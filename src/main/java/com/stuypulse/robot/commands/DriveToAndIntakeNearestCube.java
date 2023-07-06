/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.constants.Settings.CubeDetection;
import com.stuypulse.robot.constants.Settings.CubeDetection.Rotation;
import com.stuypulse.robot.constants.Settings.CubeDetection.Translation;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.robot.util.Limelight.DataType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToAndIntakeNearestCube extends CommandBase {

    // Subsystems
    private final SwerveDrive swerve;
    // private final Intake intake;

    private final Vision vision;

    // Holonomic control
    private final HolonomicController controller;
    // private final BStream aligned;

    public DriveToAndIntakeNearestCube(){
        this.swerve = SwerveDrive.getInstance();
        // this.intake = Intake.getInstance();
        this.vision = Vision.getInstance();
 
        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
        );

        SmartDashboard.putData("Vision/Controller", controller);

        // aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        // addRequirements(swerve, intake);
        addRequirements(swerve);
    }

    private boolean isAligned() {
        // these thresholds probably need to be modified
        boolean aligned = controller.isDone(
            CubeDetection.THRESHOLD_X.get(),
            CubeDetection.THRESHOLD_Y.get(),
            CubeDetection.THRESHOLD_ANGLE.get());

        SmartDashboard.putBoolean("Vision/Is Aligned ", aligned);
        return aligned;
        // return false;
    }

    @Override
    public void initialize() {
        // intake.acquire();
        LEDController.getInstance().setColor(LEDColor.BLUE, 694000000);
    }

    @Override
    public void execute() {
        Pose2d targetPose = new Pose2d(0.2, 0, new Rotation2d()); // modify x as const distance from cube
        Pose2d currentPose = new Pose2d(vision.getDistanceToCube(), 0, Rotation2d.fromDegrees(vision.getAngle()));
        // Pose2d targetPose = new Pose2d();
        // Pose2d currentPose = new Pose2d(0, 0, Rotation2d.fromDegrees(vision.getAngle()));

        swerve.setChassisSpeeds(controller.update(targetPose, currentPose));
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    public void end(boolean interupted) {
        // intake.enableBreak();
        swerve.stop();
        // intake.stop();
        LEDController.getInstance().setColor(LEDController.getInstance().getDefaultColor(), 0);
    }

}