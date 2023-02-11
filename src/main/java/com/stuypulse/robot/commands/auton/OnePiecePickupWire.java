package com.stuypulse.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.IntakeAcquireCube;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCone;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiecePickupWire extends SequentialCommandGroup{
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private HashMap<String, PathPlannerTrajectory> paths;
    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);

    public OnePiecePickupWire(){
        paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1.5 Piece + Wire", CONSTRAINTS, CONSTRAINTS),
            "1.5 Piece + Wire"
        );

        addCommands(
            new IntakeDeacquireCone(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new SwerveDriveFollowTrajectory(
                paths.get("1.5 Piece + Wire")
            ).robotRelative(),
            new IntakeAcquireCube()
        );

    }

}
