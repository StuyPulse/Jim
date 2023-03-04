package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceWire extends DebugSequentialCommandGroup {

    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double ALIGNMENT_TIME = 1.0;

    private static final PathConstraints INTAKE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints SCORE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);

    
    public TwoPieceWire() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece Wire", INTAKE_PIECE_CONSTRAINTS, SCORE_PIECE_CONSTRAINTS),
            "Intake Piece", "Score Piece", "Back Away"
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
                .alongWith(new ArmReady().alongWith(new WaitCommand(1.0).andThen(new IntakeStop()))),

            new ManagerSetScoreIndex(7),
            // new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),
            new IntakeDeacquire(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop()
        );

        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Back Away"))
                    .fieldRelative()
        );
    }
}