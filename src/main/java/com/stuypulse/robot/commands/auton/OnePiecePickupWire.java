package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.subsystems.Manager.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiecePickupWire extends SequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double INTAKE_ACQUIRE_TIME = 0.5;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);

    public OnePiecePickupWire() {
        // initial setup
        addCommands(
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE),
            new ManagerSetIntakeSide(IntakeSide.FRONT),
            new ManagerSetScoreSide(ScoreSide.OPPOSITE)
        );

        // score first piece
        addCommands(
            new ArmReady(),
            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop()
        );

        // intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),

            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("1.5 Piece + Wire", CONSTRAINTS))
                    .robotRelative()
                    .addEvent("ReadyIntakeOne", new ArmIntake().andThen(new IntakeAcquire()))
                    .withEvents(),


            new IntakeWaitForPiece().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

    }

}
