/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class SwerveDriveToPose extends CommandBase{
    private final SwerveDrive swerve;
    private final Supplier<Pose2d> targetPoses;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;

    private final FieldObject2d targetPose2d;

    public SwerveDriveToPose(Supplier<Pose2d> targetPoses){
        this.swerve = SwerveDrive.getInstance();
        this.targetPoses = targetPoses;

        controller = new HolonomicController(
            new PIDController(Translation.P, Translation.I, Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D));

        SmartDashboard.putData("Alignment/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");
        addRequirements(swerve);
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
        Odometry.USE_VISION_ANGLE.set(true);
    }

    @Override
    public void execute() {
        Pose2d currentPose = Odometry.getInstance().getPose();
        Pose2d targetPose = targetPoses.get();

        targetPose2d.setPose(targetPose);

        controller.update(targetPose, currentPose);
        swerve.setChassisSpeeds(controller.getOutput());
    }

    @Override
    public boolean isFinished(){
        return aligned.get();
    }

    public void end(boolean interupted) {
        Odometry.USE_VISION_ANGLE.set(false);
        swerve.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
    }

}
