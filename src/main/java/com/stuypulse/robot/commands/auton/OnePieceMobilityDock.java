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

public class OnePieceMobilityDock extends SequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double ENGAGE_TIME = 15;

    private static final PathConstraints MOBILITY_CONSTRAINTS = new PathConstraints(1, 1);
    private static final PathConstraints DOCK_CONSTRAINTS = new PathConstraints(1, 1);
    
    public OnePieceMobilityDock() {

        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("Dock + Mobility", MOBILITY_CONSTRAINTS, DOCK_CONSTRAINTS),
            "Mobility", "Dock"
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
            new IntakeStop()
        );

        // dock and engage
        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Mobility"))
                    .robotRelative(),

            new SwerveDriveFollowTrajectory(
                paths.get("Dock")).robotRelative(),

            new SwerveDriveAlignThenBalance().withTimeout(ENGAGE_TIME),
            new PlantEngage()
        );
    
    }

}