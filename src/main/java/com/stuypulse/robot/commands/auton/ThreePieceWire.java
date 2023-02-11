package com.stuypulse.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.IntakeAcquireCube;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCone;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCube;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceWire extends SequentialCommandGroup {
    

    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(1, 1);

    public ThreePieceWire(){
        HashMap<String, PathPlannerTrajectory> paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("3 Piece Wire", CONSTRAINTS, CONSTRAINTS),

            "Intake Piece", "Score Piece", "Intake Piece Two", "Score Piece Two"
        );

         addCommands(
            new IntakeDeacquireCone(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

         addCommands(            
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece")
            ).robotRelative(),
            new IntakeAcquireCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)
        );

        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece")
            ).fieldRelative(),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece Two")
            ).fieldRelative(),
            new IntakeAcquireCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)
        );

        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece Two")
            ).fieldRelative(),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );
    }
}