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
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

public class OnePieceMobilityDock extends DebugSequentialCommandGroup {

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
            // return new ArmTrajectory()
            //     .addState(new ArmState(dest.getShoulderState(), src.getWristState())
            //         .setShoulderTolerance(20).setWristTolerance(360))
            //     .addState(new)
            return new ArmTrajectory()
                .addState(
                    new ArmState(dest.getShoulderDegrees(), src.getWristDegrees())
                        .setWristLimp(true))
                .addState(dest);        
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

    private static final double INTAKE_DEACQUIRE_TIME = 0.5;
    private static final double STOW_WAIT_TIME = 0.5;
    private static final double REF_REACTION_TIME = 0.8;
    private static final double ENGAGE_TIME = 10.0;

    private static final PathConstraints OVER_CHARGE = new PathConstraints(1, 2);
    private static final PathConstraints DOCK = new PathConstraints(1, 2);

    public OnePieceMobilityDock() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1 Piece + Mobility Dock", OVER_CHARGE, DOCK),
            "Over Charge", "Dock"
        );

        // initial setup
        addCommands(
            new ManagerSetNodeLevel(NodeLevel.MID),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_UP),
            new ManagerSetScoreSide(ScoreSide.BACK)
        );

        // score first piece
        addCommands(
            new LEDSet(LEDColor.RED),
            new ConeAutonReady()
                // .setWristVelocityTolerance(25)
                // .setShoulderVelocityTolerance(45)
                // .withTolerance(7, 9)
                .withTimeout(3)
        );

        addCommands(
            new LEDSet(LEDColor.BLUE),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // over charge station
        addCommands(
            new LEDSet(LEDColor.GREEN),
            new ParallelDeadlineGroup(
                new SwerveDriveFollowTrajectory(paths.get("Over Charge"))
                        .robotRelative().withStop(),

                new WaitCommand(STOW_WAIT_TIME).andThen(new IntakeStop()).andThen(new FastStow().withTolerance(15, 10))
            )
        );

        // dock and engage
        addCommands(
            new LEDSet(LEDColor.PURPLE),
            new WaitCommand(REF_REACTION_TIME),
            new SwerveDriveFollowTrajectory(paths.get("Dock"))
                    .fieldRelative().withStop()
        );

        addCommands(
            new LEDSet(LEDColor.RAINBOW),

            new SwerveDriveBalanceBlay()
                .withMaxSpeed(0.6)
                .withTimeout(ENGAGE_TIME),
                // .alongWith(new FastStow().withTolerance(15, 10)),

            new PlantEngage()
        );

    }
}
