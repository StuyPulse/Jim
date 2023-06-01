/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.robot.constants.ArmTrajectories;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.constants.Settings.Alignment.Rotation;
import com.stuypulse.robot.constants.Settings.Alignment.Translation;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.Derivative;
import com.stuypulse.robot.util.HolonomicController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotAlignThenScore extends CommandBase {

    // Subsystems
    private final SwerveDrive swerve;
    private final Arm arm;
    private final Intake intake;

    private final Manager manager;

    // Holonomic control
    private final HolonomicController controller;
    private final BStream aligned;
    private boolean movingWhileScoring; // when we have aligned and ready to score and move back

    // Against grid debounce
    private final Derivative xErrorChange;
    private final BStream stoppedByGrid;

    // Logging
    private final FieldObject2d targetPose2d;

    public RobotAlignThenScore() {
        this.swerve = SwerveDrive.getInstance();
        this.arm = Arm.getInstance();
        this.intake = Intake.getInstance();
        manager = Manager.getInstance();

        controller = new HolonomicController(
            new PIDController(Translation.P,Translation.I,Translation.D),
            new PIDController(Translation.P, Translation.I, Translation.D),
            new AnglePIDController(Rotation.P, Rotation.I, Rotation.D));

        SmartDashboard.putData("Alignment/Controller", controller);

        aligned = BStream.create(this::isAligned).filtered(new BDebounceRC.Rising(Alignment.DEBOUNCE_TIME));

        xErrorChange = new Derivative();
        stoppedByGrid = BStream.create(this::isAgainstGrid).filtered(new BDebounceRC.Both(Alignment.AGAINST_GRID_DEBOUNCE));

        targetPose2d = Odometry.getInstance().getField().getObject("Target Pose");

        addRequirements(swerve, arm, intake);
    }

    private boolean isAgainstGrid() {
        return xErrorChange.getOutput() < Alignment.AGAINST_GRID_VEL_X.get();
    }

    private boolean isAligned() {
        if (Manager.getInstance().getGamePiece().isCone()) {
            return stoppedByGrid.get() || controller.isDone(
                Alignment.ALIGNED_CONE_THRESHOLD_X.get(),
                Alignment.ALIGNED_CONE_THRESHOLD_Y.get(),
                Alignment.ALIGNED_CONE_THRESHOLD_ANGLE.get());
        } else {
            return stoppedByGrid.get() || controller.isDone(
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

        xErrorChange.reset();
        
        LEDController.getInstance().setColor(LEDColor.BLUE, 694000000);
    }

    @Override
    public void execute() {
        Pose2d currentPose = Odometry.getInstance().getPose();
        Pose2d targetPose = Manager.getInstance().getScorePose();
        targetPose2d.setPose(targetPose);

        // TODO: add getError methods to controller
        xErrorChange.get(targetPose.getX() - currentPose.getX());

        controller.update(targetPose, currentPose);

        if (aligned.get() || movingWhileScoring) {
            // simply outtake when low
            if (manager.getNodeLevel() == NodeLevel.LOW) {
                intake.deacquire();
            }


            // or do scoring motion based on the game piece
            else {

                if (manager.getGamePiece().isCube()) {
                    intake.deacquire();
                } else if (manager.getGamePiece() == GamePiece.CONE_TIP_OUT) {
                    arm.setTargetState(
                        Manager.getInstance().getNodeLevel() == NodeLevel.MID ?
                            ArmTrajectories.Score.Mid.kConeTipOutFront :
                            ArmTrajectories.Score.High.kConeTipOutFront);

                    if (arm.isAtTargetState(Settings.Score.kShoulderTipOutTolerance.get(), 360)) {
                        intake.enableCoast();
                        movingWhileScoring = true;
                        swerve.setChassisSpeeds(
                            new ChassisSpeeds(
                                -Units.inchesToMeters(Settings.Score.kBackwardsTipOutSpeed.get()),
                                0,
                                0));
                    }
                } else {
                    // don't automate yet
                }

            }

        } else {
            swerve.setChassisSpeeds(controller.getOutput());
        }
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
