package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceWithPlant;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPiecePickupDock extends SequentialCommandGroup {

    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double ALIGNMENT_TIME = 1.0;
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
            new LEDSet(LEDColor.RAINBOW),
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_IN),
            new ManagerSetIntakeSide(IntakeSide.FRONT),
            new ManagerSetScoreSide(ScoreSide.OPPOSITE)
        );

        // score first piece
        addCommands(
            new LEDSet(LEDColor.YELLOW.pulse()),
            new ArmReady(),
            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new LEDSet(LEDColor.RAINBOW),
            new ArmNeutral()
        );

        // drive to and intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetNodeLevel(NodeLevel.MID),
            
            new LEDSet(LEDColor.GREEN),
            new SwerveDriveFollowTrajectory(
                paths.get("Intake One"))
                    .robotRelative()
                    .addEvent("ReadyIntakeOne", new ArmIntake().andThen(new IntakeAcquire()))
                    .withEvents(), 

            new LEDSet(LEDColor.YELLOW),
            new IntakeWaitForPiece().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new LEDSet(LEDColor.RAINBOW),
            new ArmNeutral()
        );
        
        // drive to grid and score second piece
        addCommands( 
            new LEDSet(LEDColor.GREEN),
            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece"))
                    .fieldRelative()
                    .addEvent("ReadyArmOne", new ArmReady())
                    .withEvents(),

            new ManagerSetScoreIndex(1),
            new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),

            new LEDSet(LEDColor.YELLOW.pulse()),
            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new LEDSet(LEDColor.RAINBOW),
            new ArmNeutral()
        );

        // intake third piece
        addCommands(
            new LEDSet(LEDColor.GREEN),
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Two"))
                    .fieldRelative()
                    .addEvent("ReadyIntakeTwo", new ArmIntake().andThen(new IntakeAcquire()))
                    .withEvents(),

            new LEDSet(LEDColor.YELLOW),
            new IntakeWaitForPiece().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new LEDSet(LEDColor.RAINBOW),
            new ArmNeutral()
        );

        // dock and engage
        addCommands(
            new LEDSet(LEDColor.GREEN),
            new SwerveDriveFollowTrajectory(
                paths.get("Dock"))
                    .fieldRelative()
                    .addEvent("ArmNeutral", new ArmNeutral())
                    .withEvents(),


            new SwerveDriveBalanceWithPlant().withTimeout(ENGAGE_TIME),
            new LEDSet(LEDColor.RED.pulse()),
            new PlantEngage(),
            new LEDSet(LEDColor.BLUE)
        );
    }
}
