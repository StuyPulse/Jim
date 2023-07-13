/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands;

import static com.stuypulse.robot.constants.Settings.CubeDetection.*;

import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToAndIntakeNearestCube extends CommandBase {

    // Subsystems
    private final SwerveDrive swerve;
    private final Intake intake;

    private final Vision vision;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    public DriveToAndIntakeNearestCube(){
        this.swerve = SwerveDrive.getInstance();
        this.intake = Intake.getInstance();
        this.vision = Vision.getInstance();
 
        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
        );

        SmartDashboard.putData("Vision/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(DEBOUNCE_TIME));
        SmartDashboard.putBoolean("Vision/Is Aligned ", aligned.get());

        addRequirements(swerve, intake);
    }

    private boolean isAligned() {
        boolean aligned = controller.isDone(
            THRESHOLD_X.get(),
            THRESHOLD_Y.get(),
            THRESHOLD_ANGLE.get());

        return aligned;
    }

    @Override
    public void initialize() {
        LEDController.getInstance().setColor(LEDColor.BLUE, 694000000);
    }

    @Override
    public void execute() {
        double cubeDistance  = vision.getDistanceToCube();
        Rotation2d cubeRotation = vision.getRotationToObject();

        Pose2d targetPose = new Pose2d(TARGET_CUBE_DISTANCE.get(), 0, new Rotation2d());
        Pose2d currentPose = new Pose2d(cubeDistance * cubeRotation.getCos(), cubeDistance * cubeRotation.getSin(), cubeRotation);

        swerve.setChassisSpeeds(controller.update(targetPose, currentPose));

        if(aligned.get() && !intake.isAcquiring()) {
            intake.acquire();
        }
    }

    @Override
    public boolean isFinished() {
        return aligned.get() && intake.hasGamePiece();
    }

    public void end(boolean interupted) {
        intake.stop();
        swerve.stop();

        LEDController.getInstance().setColor(LEDController.getInstance().getDefaultColor(), 0);
    }

}