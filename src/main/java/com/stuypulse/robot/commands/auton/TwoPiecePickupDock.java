package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveAlignThenBalance;
import com.stuypulse.robot.subsystems.Manager.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPiecePickupDock extends SequentialCommandGroup {

    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double ENGAGE_TIME = 3.0;

    private static final PathConstraints INTAKE_ONE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints SCORE_ONE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints INTAKE_TWO_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints DOCK_CONSTRAINTS = new PathConstraints(2, 2);
    
    public TwoPiecePickupDock() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2.5 Piece + Dock", INTAKE_ONE_PIECE_CONSTRAINTS, SCORE_ONE_PIECE_CONSTRAINTS, INTAKE_TWO_PIECE_CONSTRAINTS, DOCK_CONSTRAINTS),
            "Intake One", "Score Piece", "Intake Two", "Dock"
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

        // drive to and intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetNodeLevel(NodeLevel.HIGH),

            new SwerveDriveFollowTrajectory(
                paths.get("Intake One"))
                    .robotRelative()
                    .addEvent("ReadyIntakeOne", new ArmIntake())
                    .withEvents(), 
            new IntakeAcquire().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );
        
        // drive to grid and score second piece
        addCommands( 
            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece"))
                    .fieldRelative()
                    .addEvent("ReadyArmOne", new ArmReady())
                    .withEvents(),

            new ManagerSetScoreIndex(1),

            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        // intake third piece
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Two"))
                    .fieldRelative()
                    .addEvent("ReadyIntakeTwo", new ArmIntake())
                    .withEvents(),
            new IntakeAcquire().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        // dock and engage
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Dock"))
                    .fieldRelative()
                    .addEvent("ArmNeutral", new ArmNeutral())
                    .withEvents(),


            new SwerveDriveAlignThenBalance().withTimeout(ENGAGE_TIME),
            new PlantEngage()
        );
    }
}
