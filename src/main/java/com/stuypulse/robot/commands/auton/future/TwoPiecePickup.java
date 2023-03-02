package com.stuypulse.robot.commands.auton.future;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.subsystems.Manager.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPiecePickup extends SequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double INTAKE_ACQUIRE_TIME = 1.0;
    private static final double ALIGNMENT_TIME = 1.0;

    private static final PathConstraints INTAKE_ONE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints SCORE_ONE_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    private static final PathConstraints INTAKE_TWO_PIECE_CONSTRAINTS = new PathConstraints(2, 2);
    
    public TwoPiecePickup() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2.5 Piece", INTAKE_ONE_PIECE_CONSTRAINTS, SCORE_ONE_PIECE_CONSTRAINTS, INTAKE_TWO_PIECE_CONSTRAINTS),
            
            "Intake One", "Score Piece", "Intake Two"
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
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmStow()
        );

        // drive to and intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetNodeLevel(NodeLevel.HIGH),

            new SwerveDriveFollowTrajectory(
                paths.get("Intake One"))
                    .robotRelative()
                    .addEvent("ReadyIntakeOne",new ArmIntake())
                    .withEvents(),
            new IntakeAcquire().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmStow()
        );
        
        // drive to grid and score second piece
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece"))
                    .fieldRelative()
                    .addEvent("ReadyArmOne", new ArmReady())
                    .withEvents(),

            new ManagerSetScoreIndex(1),
            new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),

            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmStow()
        );

        // intake third piece
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Two"))
                    .fieldRelative()
                    .addEvent("ReadyIntakeTwo", new ArmIntake().andThen(new IntakeAcquire()))
                    .withEvents(),

            new IntakeAcquire().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmStow()
        );
    }
}
