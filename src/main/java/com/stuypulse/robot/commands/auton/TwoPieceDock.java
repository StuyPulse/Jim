package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceBlay;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceWithPlant;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceDock extends DebugSequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 0.2;
    private static final double CUBE_DEACQUIRE_TIME = 0.2;
    private static final double INTAKE_STOP_WAIT_TIME = 0.4;
    private static final double INTAKE_WAIT_TIME = 0.2;
    private static final double ACQUIRE_WAIT_TIME = 0.2;
    private static final double ENGAGE_TIME = 10.0;
    private static final double STOW_WAIT_TIME = 0.2;  

    private static final PathConstraints INTAKE_PIECE_CONSTRAINTS = new PathConstraints(3.7, 3.5);
    private static final PathConstraints SCORE_PIECE_CONSTRAINTS = new PathConstraints(4.5, 4);
    private static final PathConstraints DOCK_CONSTRAINTS = new PathConstraints(2.2, 2);

    public TwoPieceDock() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece + Dock", INTAKE_PIECE_CONSTRAINTS, SCORE_PIECE_CONSTRAINTS, DOCK_CONSTRAINTS),
            "Intake Piece", "Score Piece", "Dock"
        );

        var arm = Arm.getInstance();

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
                .withTolerance(7, 9)
                .withTimeout(4)
        );

        addCommands(
            new LEDSet(LEDColor.BLUE),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),

            new LEDSet(LEDColor.GREEN),

            new ParallelCommandGroup(
                new SwerveDriveFollowTrajectory(paths.get("Intake Piece"))
                    .robotRelative(),

                new WaitCommand(INTAKE_STOP_WAIT_TIME)
                    .andThen(new IntakeStop())
                    .andThen(new WaitCommand(INTAKE_WAIT_TIME))
                    .andThen(new IntakeAcquire()),

                new ArmIntake()
                    .withTolerance(7, 10)
                    .withTimeout(6.5)
            ),

            new WaitCommand(ACQUIRE_WAIT_TIME)
                .alongWith(arm.runOnce(() -> arm.setWristVoltage(-2))),

            arm.runOnce(() -> arm.setWristVoltage(0))
        );
        
        // drive to grid and score second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),
            new ManagerSetScoreSide(ScoreSide.BACK),

            new LEDSet(LEDColor.RED),

            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece"))
                    .fieldRelative()
                .alongWith(new WaitCommand(1.0).andThen(new ArmReady().alongWith(new WaitCommand(0.1).andThen(new IntakeStop())))),

            new ManagerSetScoreIndex(1),
            // new SwerveDriveToScorePose().withTimeout(ALIGNMENT_TIME),
            new IntakeDeacquire(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop()
        );
        
        // dock and engage
        addCommands(
            new LEDSet(LEDColor.PURPLE),
            new ParallelCommandGroup(
                new SwerveDriveFollowTrajectory(paths.get("Dock"))
                        .fieldRelative(),

                new WaitCommand(STOW_WAIT_TIME).andThen(new ArmStow())
            )
        );

        addCommands(
            new LEDSet(LEDColor.RAINBOW),

            new SwerveDriveBalanceBlay()
                .withMaxSpeed(1.3)
                .withTimeout(ENGAGE_TIME),

            new PlantEngage()
        );
    }

}