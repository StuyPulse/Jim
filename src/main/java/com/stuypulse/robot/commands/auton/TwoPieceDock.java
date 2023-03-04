package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceBlay;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceWithPlant;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceDock extends DebugSequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double INTAKE_ACQUIRE_TIME = 1.5;
    private static final double ENGAGE_TIME = 10.0;

    private static final PathConstraints INTAKE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints SCORE_PIECE_CONSTRAINTS = new PathConstraints(3,2);
    private static final PathConstraints DOCK_CONSTRAINTS = new PathConstraints(1, 2);

    public TwoPieceDock() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece + Dock", INTAKE_PIECE_CONSTRAINTS, SCORE_PIECE_CONSTRAINTS, DOCK_CONSTRAINTS),
            "Intake Piece", "Score Piece", "Dock"
        );

        // initial setup
        addCommands(
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_UP),
            new ManagerSetScoreSide(ScoreSide.BACK)
        );

        // score first piece
        addCommands(
            new ArmReady().withTolerance(7, 7).withTimeout(4),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),

            new ParallelCommandGroup(new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece"))
                    .robotRelative(),
                new IntakeAcquire(),
                new ArmIntake()
            )
        );
        
        // drive to grid and score second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetScoreSide(ScoreSide.BACK),

            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece"))
                    .fieldRelative()
                .alongWith(new IntakeStop().andThen(new ArmReady())),

            new ManagerSetScoreIndex(1),
            // new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),
            new IntakeDeacquire(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop()
        );

        // dock and engage
        addCommands(
            // new ManagerSetScoreSide(ScoreSide.FRONT),

            new SwerveDriveFollowTrajectory(
                paths.get("Dock"))
                    .fieldRelative()
                // .alongWith(new ArmStow()),
                .alongWith(new ArmStow()),

            new SwerveDriveBalanceBlay().withMaxSpeed(1.0).withTimeout(ENGAGE_TIME),
            new PlantEngage()
        );
    }

}