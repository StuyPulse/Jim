/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceBlay;
import com.stuypulse.robot.constants.ArmTrajectories.Ready;
import com.stuypulse.robot.constants.Settings.Arm.Shoulder;
import com.stuypulse.robot.constants.Settings.Arm.Wrist;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

public class TwoPieceDock extends DebugSequentialCommandGroup {
    static class AutonReady extends ArmRoutine {
        public AutonReady() {
            super(() -> {
                if (Manager.getInstance().getNodeLevel() == NodeLevel.HIGH) {
                    return Ready.High.kCubeAutonBack;
                } else {
                    return Ready.Mid.kAutonCubeBack;
                }
            });
        }

        @Override
        protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
            double wristSafeAngle = Wrist.WRIST_SAFE_ANGLE.get();

            return new ArmTrajectory()
                .addState(new ArmState(src.getShoulderDegrees(), wristSafeAngle)
                    .setWristTolerance(45))
                .addState(new ArmState(dest.getShoulderState(), dest.getWristState())
                    .setWristTolerance(23).setShoulderTolerance(20));
        }
    }
    private class ArmReadyBOOM extends ArmRoutine {
        public ArmReadyBOOM() {
            super(Manager.getInstance()::getReadyTrajectory);
        }

        @Override
        protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
            double wristSafeAngle = Wrist.WRIST_SAFE_ANGLE.get();

            return new ArmTrajectory()
                .addState(new ArmState(src.getShoulderDegrees(), wristSafeAngle)
                    .setWristTolerance(30))
                .addState(
                    new ArmState(dest.getShoulderDegrees(), wristSafeAngle).setWristLimp(true).setWristTolerance(360))
                .addState(dest);

            // return new ArmTrajectory().addState(dest);
        }
    }

    private class FastStow extends ArmRoutine {
        public FastStow() {
            super(() -> new ArmState(-90, 90)
                            .setWristTolerance(360));
        }

        @Override
        protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
            return new ArmTrajectory()
                .addState(dest);
        }
    }

    private static final double INTAKE_DEACQUIRE_TIME = 0.2;
    private static final double CUBE_DEACQUIRE_TIME = 0.2;
    private static final double INTAKE_STOP_WAIT_TIME = 1;
    private static final double INTAKE_WAIT_TIME = 0.2;
    private static final double ACQUIRE_WAIT_TIME = 0.1;
    private static final double ENGAGE_TIME = 10.0;
    private static final double STOW_WAIT_TIME = 0;

    private static final PathConstraints INTAKE_PIECE_CONSTRAINTS = new PathConstraints(2.2, 2);
    private static final PathConstraints SCORE_PIECE_CONSTRAINTS = new PathConstraints(4.2, 3.5);
    private static final PathConstraints DOCK_CONSTRAINTS = new PathConstraints(1, 2);

    public TwoPieceDock() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece + Dock", INTAKE_PIECE_CONSTRAINTS, SCORE_PIECE_CONSTRAINTS, DOCK_CONSTRAINTS),
            "Intake Piece", "Score Piece", "Dock"
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
            new ArmReady()
                .withTolerance(7, 9)
                .setShoulderVelocityTolerance(25)
                .setWristVelocityTolerance(45)
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

        // intake second piece
        addCommands(

            new LEDSet(LEDColor.GREEN),

            new ParallelDeadlineGroup(
                new SwerveDriveFollowTrajectory(paths.get("Intake Piece"))
                    .robotRelative().withStop(),

                new WaitCommand(INTAKE_STOP_WAIT_TIME)
                    .andThen(new IntakeStop())
                    .andThen(new WaitCommand(INTAKE_WAIT_TIME))
                    .andThen(new ManagerSetGamePiece(GamePiece.CUBE))
                    .andThen(new IntakeAcquire()),

                new ArmIntakeBOOM()
                    .withTolerance(12, 10)
                    .withTimeout(6.5)
            ),

            new WaitCommand(ACQUIRE_WAIT_TIME)
                .alongWith(arm.runOnce(() -> arm.setWristVoltage(-2))),

            arm.runOnce(() -> arm.setWristVoltage(0))
        );

        addCommands(
            arm.runOnce(() -> {
                arm.setShoulderConstraints(Shoulder.AUTON_MAX_VELOCITY, Shoulder.AUTON_MAX_ACCELERATION);
                arm.setWristConstraints(Wrist.TELEOP_MAX_VELOCITY, Wrist.TELEOP_MAX_ACCELERATION);
            })
        );

        // drive to grid and score second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetScoreSide(ScoreSide.BACK),

            new LEDSet(LEDColor.RED),

            arm.runOnce(() -> arm.setShoulderVelocityFeedbackCutoff(10.0)),

            new ParallelCommandGroup(
                new SwerveDriveFollowTrajectory(
                    paths.get("Score Piece"))
                        .fieldRelative().withStop(),

                new WaitCommand(0.2)
                    .andThen(new AutonReady()
                        .withTimeout(paths.get("Score Piece").getTotalTimeSeconds() + 0.5)),

                new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    new IntakeStop()
                )
            ),

            // new ManagerSetScoreIndex(1),
            new IntakeDeacquire(),
            // new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop()
        );

        // dock and engage
        addCommands(
            new LEDSet(LEDColor.PURPLE),
            new ParallelDeadlineGroup(
                new SwerveDriveFollowTrajectory(paths.get("Dock"))
                        .fieldRelative().withStop(),

                new WaitCommand(STOW_WAIT_TIME).andThen(new ArmStow())
            )
        );

        addCommands(
            new LEDSet(LEDColor.GREEN),

            new SwerveDriveBalanceBlay()
                .withMaxSpeed(0.8)
                .withTimeout(ENGAGE_TIME)
                .alongWith(new FastStow().withTolerance(15, 10)),

            new PlantEngage()
        );
    }

}
