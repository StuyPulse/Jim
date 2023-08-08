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
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

public class OnePieceDock extends DebugSequentialCommandGroup {

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

    private static final double INTAKE_DEACQUIRE_TIME = 0.5;
    private static final double INTAKE_ACQUIRE_TIME = 0.8;
    private static final double ENGAGE_TIME = 10.0;

    private static final PathConstraints DOCK = new PathConstraints(1.8, 2.5);

    public OnePieceDock() {

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
                .setWristVelocityTolerance(25)
                .setShoulderVelocityTolerance(45)
                .withTolerance(7, 9)
                .withTimeout(4)
        );

        addCommands(
            new LEDSet(LEDColor.BLUE),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // dock and engage
        addCommands(
            new LEDSet(LEDColor.PURPLE),
            new ParallelDeadlineGroup(
                new SwerveDriveFollowTrajectory(PathPlanner.loadPath("1 Piece + Dock", DOCK))
                        .robotRelative().withStop(),

                new WaitCommand(INTAKE_ACQUIRE_TIME).andThen(new IntakeStop()).andThen(new ArmStow())
            )
        );

        addCommands(
            new LEDSet(LEDColor.RAINBOW),

            new SwerveDriveBalanceBlay(0.6)
                .withTimeout(ENGAGE_TIME)
                .alongWith(new FastStow().withTolerance(15, 10)),

            new PlantEngage()
        );

    }
}
