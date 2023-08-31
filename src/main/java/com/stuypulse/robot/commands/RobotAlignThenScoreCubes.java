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
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotAlignThenScoreCubes extends CommandBase {

    // Subsystems
    private final SwerveDrive swerve;
    private final Intake intake;

    private final Manager manager;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;
    private boolean movingWhileScoring; // when we have aligned and ready to score and move back

    // Logging
    private final FieldObject2d targetPose2d;

    public RobotAlignThenScoreCubes(){
        this.swerve = SwerveDrive.getInstance();
        this.intake = Intake.getInstance();
        manager = Manager.getInstance();

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
        if(Manager.getInstance().getNodeLevel() == Manager.NodeLevel.LOW) {
            SmartDashboard.putString("Alignment/Mode", "LOW");
            return controller.isDone(
                Alignment.ALIGNED_LOW_THRESHOLD_X.get(),
                Alignment.ALIGNED_LOW_THRESHOLD_Y.get(),
                Alignment.ALIGNED_LOW_THRESHOLD_ANGLE.get());
        }
        else if (Manager.getInstance().getGamePiece().isCone()) {
            SmartDashboard.putString("Alignment/Mode", "CONE");
            return controller.isDone(
                Alignment.ALIGNED_CONE_THRESHOLD_X.get(),
                Alignment.ALIGNED_CONE_THRESHOLD_Y.get(),
                Alignment.ALIGNED_CONE_THRESHOLD_ANGLE.get());
        } else {
            SmartDashboard.putString("Alignment/Mode", "CUBE");
            return controller.isDone(
                Alignment.ALIGNED_CUBE_THRESHOLD_X.get(),
                Alignment.ALIGNED_CUBE_THRESHOLD_Y.get(),
                Alignment.ALIGNED_CUBE_THRESHOLD_ANGLE.get());
        }
    }

    @Override
    public void initialize() {
        movingWhileScoring = false;
        intake.enableBreak();
        Odometry.USE_VISION_ANGLE.set(true);

        LEDController.getInstance().setColor(LEDColor.BLUE, 694000000);
    }

    @Override
    public void execute() {
        Pose2d currentPose = Odometry.getInstance().getPose();
        Pose2d targetPose = Manager.getInstance().getScorePose();
        targetPose2d.setPose(targetPose);

        controller.update(targetPose, currentPose);

        if (aligned.get() || movingWhileScoring) {
            if (!movingWhileScoring) {
                swerve.stop();
            }

            // simply outtake when low
            if (manager.getNodeLevel() == NodeLevel.LOW) {
                LEDController.getInstance().setColor(LEDColor.GREEN, 694000000);
                intake.deacquire();
            }


            // or do scoring motion based on the game piece
            else {

                // only score for cubes
                if (manager.getGamePiece().isCube()) {
                    LEDController.getInstance().setColor(LEDColor.GREEN, 694000000);
                    intake.deacquire();
                }
            }

        } else {
            swerve.setChassisSpeeds(controller.getOutput());
        }

        SmartDashboard.putBoolean("Alignment/Aligned", isAligned());
        SmartDashboard.putBoolean("Alignment/Aligned Debounced", aligned.get());
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public void end(boolean interupted) {
        intake.enableBreak();
        Odometry.USE_VISION_ANGLE.set(false);
        swerve.stop();
        intake.stop();
        targetPose2d.setPose(Double.NaN, Double.NaN, new Rotation2d(Double.NaN));
        
        LEDController.getInstance().setColor(LEDController.getInstance().getDefaultColor(), 0);
    }

}
