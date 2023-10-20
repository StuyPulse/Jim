/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

public class TwoPieceBump extends DebugSequentialCommandGroup {

    static class ConeAutonReady extends ArmRoutine {
        public ConeAutonReady() {
            super(() -> {
                if (Manager.getInstance().getNodeLevel() == NodeLevel.HIGH)
                    return new ArmState(-179, 136 - 8);
                else
                    return new ArmState(-161.7, 133.9);
            });
        }

        @Override   
        protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
            return new ArmTrajectory()
                .addState(
                    new ArmState(dest.getShoulderDegrees(), src.getWristDegrees())
                        .setWristLimp(true))
                .addState(dest);
        }
    }

    private static final double INTAKE_DEACQUIRE_TIME = 0.5;
    private static final double INTAKE_STOP_WAIT_TIME = 1;
    private static final double INTAKE_ACQUIRE_TIME = 0.5;
    private static final double INTAKE_WAIT_TIME = 2.0;
    private static final double ACQUIRE_WAIT_TIME = 0.4;
    private static final double READY_WAIT_TIME = 0.5;

    private static final PathConstraints INTAKE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints SCORE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);


    public TwoPieceBump() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece Bump", INTAKE_PIECE_CONSTRAINTS, SCORE_PIECE_CONSTRAINTS),
            "Intake Piece", "Score Piece", "Back Away"
        );

        var arm = Arm.getInstance();


        // initial setup
        addCommands(
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_UP),
            new ManagerSetScoreSide(ScoreSide.BACK)
        );

        // score first piece
        addCommands(
            new LEDSet(LEDColor.RED),
            new ConeAutonReady()
                .withTimeout(3)
        );

        addCommands(
            arm.runOnce(() -> {
                arm.setShoulderConstraints(Shoulder.TELEOP_MAX_VELOCITY, Shoulder.TELEOP_MAX_ACCELERATION);
                arm.setWristConstraints(Wrist.TELEOP_MAX_VELOCITY, Wrist.TELEOP_MAX_ACCELERATION);
            })
        );

        addCommands(
            new LEDSet(LEDColor.BLUE),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // intake second piece and strafe
        addCommands(
            new LEDSet(LEDColor.GREEN),

            new ParallelDeadlineGroup(
                new SwerveDriveFollowTrajectory(paths.get("Intake Piece"))
                    .robotRelative().withStop(),

                new IntakeScore()
                    .andThen(new WaitCommand(INTAKE_STOP_WAIT_TIME))
                    .andThen(new IntakeStop())
                    .andThen(new WaitCommand(INTAKE_WAIT_TIME))
                    .andThen(new ManagerSetGamePiece(GamePiece.CUBE))
                    .andThen(new IntakeAcquire()),

                new ArmIntakeBOOM()
                    .withTolerance(12, 10)
                    .withTimeout(6.5)
            ),

            new WaitCommand(ACQUIRE_WAIT_TIME).until(Intake.getInstance()::hasGamePiece)
                .alongWith(arm.runOnce(() -> arm.setWristVoltage(-2))),

            arm.runOnce(() -> arm.setWristVoltage(0))
        );

        addCommands(
            arm.runOnce(() -> {
                arm.setShoulderVelocityFeedbackCutoff(20);
                arm.setShoulderVelocityFeedbackDebounce(0.0);
            })
        );

        // drive to grid and score second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetScoreSide(ScoreSide.BACK),

            new LEDSet(LEDColor.RED),

            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece"))
                    .fieldRelative()
                    .withStop()
                .alongWith(new WaitCommand(READY_WAIT_TIME).andThen(new ArmReady()))
                .alongWith(new WaitCommand(INTAKE_ACQUIRE_TIME).andThen(new IntakeStop())),

            new ManagerSetGridNode(7),
            // new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),
            new IntakeDeacquire(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop()
        );

        addCommands(
            new LEDSet(LEDColor.RAINBOW),
            new SwerveDriveFollowTrajectory(
                paths.get("Back Away"))
                    .withStop()
                    .fieldRelative()
        );
    }
}
