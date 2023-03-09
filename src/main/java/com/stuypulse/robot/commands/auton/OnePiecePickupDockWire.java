package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveAlignThenBalance;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceBlay;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.DebugSequentialCommandGroup;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// The best
public class OnePiecePickupDockWire extends DebugSequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 0.5;
    private static final double INTAKE_ACQUIRE_TIME = 1.5;
    private static final double INTAKE_STOP_WAIT_TIME = 0.5;
    private static final double INTAKE_WAIT_TIME = 2.0;
    private static final double ENGAGE_TIME = 10.0;
    private static final double ACQUIRE_WAIT_TIME = 0.4;

    private static final PathConstraints INTAKE_PIECE = new PathConstraints(3, 2);
    private static final PathConstraints DOCK = new PathConstraints(1, 2);

    public OnePiecePickupDockWire() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1.5 Piece + Dock Wire", INTAKE_PIECE, DOCK),
            "Intake Piece", "Dock"
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
                    .robotRelative()
                    .withStop(),

                new WaitCommand(INTAKE_STOP_WAIT_TIME)
                    .andThen(new IntakeStop())
                    .andThen(new WaitCommand(INTAKE_WAIT_TIME))
                    .andThen(new IntakeAcquire()),

                new ArmIntake()
                    .withTolerance(7, 10)
                    .withTimeout(4.5)
            ),

            new WaitCommand(ACQUIRE_WAIT_TIME)
                .alongWith(arm.runOnce(() -> arm.setWristVoltage(-2))),

            arm.runOnce(() -> arm.setWristVoltage(0))
        );
        
        // dock and engage
        addCommands(
            new LEDSet(LEDColor.PURPLE),
                new ParallelCommandGroup(
                    new SwerveDriveFollowTrajectory(paths.get("Dock"))
                            .fieldRelative(),

                    new WaitCommand(INTAKE_ACQUIRE_TIME).andThen(new IntakeStop()).andThen(new ArmStow())
                )
        );

        addCommands(
            new LEDSet(LEDColor.RAINBOW),

            new SwerveDriveBalanceBlay()
                .withMaxSpeed(1.0)
                .withTimeout(ENGAGE_TIME),

            new PlantEngage()
        );
    
    }
}
