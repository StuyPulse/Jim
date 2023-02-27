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

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiecePickupDock extends SequentialCommandGroup{

    private static final double INTAKE_DEACQUIRE_TIME = 0.3;
    private static final double INTAKE_ACQUIRE_TIME = 3.5;
    private static final double ENGAGE_TIME = 10.0;

    private static final PathConstraints INTAKE_PIECE = new PathConstraints(1.5, 2);
    private static final PathConstraints DOCK = new PathConstraints(3, 2);

    public OnePiecePickupDock() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1.5 Piece + Dock", INTAKE_PIECE, DOCK),
            "Intake Piece", "Dock" 
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
            // new ArmScore(),
            new IntakeScore().withTimeout(INTAKE_DEACQUIRE_TIME)
        );

        // intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),

            new ParallelCommandGroup(new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece"))
                    .robotRelative(),
                new IntakeAcquire().andThen(new ArmIntake())
            ),

            new IntakeAcquire().withTimeout(2),
            new IntakeStop()
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
