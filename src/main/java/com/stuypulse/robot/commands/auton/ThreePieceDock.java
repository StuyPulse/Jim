package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceWithPlant;
import com.stuypulse.robot.subsystems.Manager.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceDock extends SequentialCommandGroup {

    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double ALIGNMENT_TIME = 1.0;
    private static final double ENGAGE_TIME = 3.0;

    private static final PathConstraints INTAKE_PIECE_TWO = new PathConstraints(4, 3);
    private static final PathConstraints SCORE_PIECE_TWO = new PathConstraints(3, 2);
    private static final PathConstraints INTAKE_PIECE_THREE = new PathConstraints(4, 3);
    private static final PathConstraints SCORE_PIECE_THREE = new PathConstraints(3, 2);
    private static final PathConstraints DOCK = new PathConstraints(4, 3);

    public ThreePieceDock() {
        // load paths into hashmap
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("3 Piece + Dock", INTAKE_PIECE_TWO, SCORE_PIECE_TWO, INTAKE_PIECE_THREE, SCORE_PIECE_THREE, DOCK),

            "Intake Piece Two", "Score Piece Two", "Intake Piece Three", "Score Piece Three", "Dock"
        );

        // initial setup
        addCommands(
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_UP),
            new ManagerSetScoreSide(ScoreSide.BACK)
        );

        // score first piece
        addCommands(
            new ArmReady(),
            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        // drive to second game piece and intake
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetNodeLevel(NodeLevel.HIGH),

            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece Two"))
                    .robotRelative()
                    .addEvent("ReadyIntakeOne", new ArmIntake().andThen(new IntakeAcquire()))
                    .withEvents(),

            new IntakeAcquire().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );
        
        // drive to grid and score second piece
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece Two"))
                    .fieldRelative()
                    .addEvent("ReadyArmOne", new ArmReady().andThen(new ArmScore()))
                    .withEvents(),


            new ManagerSetScoreIndex(1),
            new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),

            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        // drive to and intake third piece
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece Three"))
                    .robotRelative()
                    .addEvent("ReadyIntakeTwo", new ArmIntake().andThen(new IntakeAcquire()))
                    .withEvents(),
            new IntakeAcquire().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        // drive to grid and score third piece
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece Three"))
                    .fieldRelative()
                    .addEvent("ReadyArmTwo", new ArmReady())
                    .withEvents(),

            new ManagerSetScoreIndex(4),
            new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),

            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        // dock and engage
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Dock"))
                    .fieldRelative()
                    .addEvent("ArmNeutral", new ArmNeutral()),

            new SwerveDriveBalanceWithPlant().withTimeout(ENGAGE_TIME),
            new PlantEngage()
        );
    }
}