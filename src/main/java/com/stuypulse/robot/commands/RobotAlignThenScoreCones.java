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
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.robot.util.Limelight.DataType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotAlignThenScoreCones extends CommandBase {

    // Subsystems
    private final SwerveDrive swerve;
    private final Intake intake;

    private final Vision vision;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    // Logging
    private final FieldObject2d targetPose2d;

    public RobotAlignThenScoreCones(){
        this.swerve = SwerveDrive.getInstance();
        this.intake = Intake.getInstance();
        this.vision = Vision.getInstance();

        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D));

        SmartDashboard.putData("Alignment/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");
        addRequirements(swerve, intake);
    }

    private boolean isAligned() {
        if (Manager.getInstance().getGamePiece().isCone()) {
            return controller.isDone(
                Alignment.ALIGNED_CONE_THRESHOLD_X.get(),
                Alignment.ALIGNED_CONE_THRESHOLD_Y.get(),
                Alignment.ALIGNED_CONE_THRESHOLD_ANGLE.get());
        } else {
            return controller.isDone(
                Alignment.ALIGNED_CUBE_THRESHOLD_X.get(),
                Alignment.ALIGNED_CUBE_THRESHOLD_Y.get(),
                Alignment.ALIGNED_CUBE_THRESHOLD_ANGLE.get());
        }
    }

    @Override
    public void initialize() {
        intake.enableBreak();
        Odometry.USE_VISION_ANGLE.set(true);

        vision.setPipeline(DataType.TAPE);

        LEDController.getInstance().setColor(LEDColor.BLUE, 694000000);
    }

    @Override
    public void execute() {
        Pose2d currentPose = Odometry.getInstance().getPose();
        Pose2d targetPose = new Pose2d(
            currentPose.getTranslation().getX() + vision.getDistance() * Math.cos(vision.getAngle()),
            currentPose.getTranslation().getY() + vision.getDistance() * Math.sin(vision.getAngle()),
            Rotation2d.fromDegrees(0)
        );
        targetPose2d.setPose(targetPose);

        controller.update(targetPose, currentPose);
    }

    @Override
    public boolean isFinished(){
        return isAligned();
    }

    public CommandBase scoreCone() {
        return this.andThen(new RobotScore());
    }

    public void end(boolean interupted) {
        intake.enableBreak();
        Odometry.USE_VISION_ANGLE.set(false);
        swerve.stop();
        intake.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));

        vision.setPipeline(DataType.APRIL_TAG);
        
        LEDController.getInstance().setColor(LEDController.getInstance().getDefaultColor(), 0);
    }

}