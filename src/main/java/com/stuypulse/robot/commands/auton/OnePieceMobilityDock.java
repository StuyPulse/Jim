package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceBlay;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceMobilityDock extends DebugSequentialCommandGroup {

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
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_UP),
            new ManagerSetScoreSide(ScoreSide.BACK)
        );

        // score first piece
        addCommands(

            new ArmReady()
                .setWristVelocityTolerance(25)
                .setShoulderVelocityTolerance(45)
                .withTolerance(7, 9)
                .withTimeout(6)
        );

        addCommands(
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // over charge station
        addCommands(
            new ParallelDeadlineGroup(
                new SwerveDriveFollowTrajectory(paths.get("Over Charge"))
                        .robotRelative().withStop(),

                new WaitCommand(STOW_WAIT_TIME).andThen(new IntakeStop()).andThen(new FastStow().withTolerance(15, 10))
            )
        );
        
        // dock and engage
        addCommands(
            new WaitCommand(REF_REACTION_TIME),
            new SwerveDriveFollowTrajectory(paths.get("Dock"))
                    .fieldRelative().withStop()
        );

        addCommands(
            new SwerveDriveBalanceBlay()
                .withMaxSpeed(0.6)
                .withTimeout(ENGAGE_TIME),
                // .alongWith(new FastStow().withTolerance(15, 10)),

            new PlantEngage()
        );
    
    }
}
