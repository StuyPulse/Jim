package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCube;
import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveEngage;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceDock extends SequentialCommandGroup {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(5, 3);
    private HashMap<String, PathPlannerTrajectory> paths;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    public OnePieceDock() {
        paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1 Piece + Dock", CONSTRAINTS, CONSTRAINTS),
            "1 Piece + Dock"
        );

        addCommands(
            // new ArmFollowTrajectory(),
            // new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new SwerveDriveFollowTrajectory(
                paths.get("1 Piece + Dock")
            ).robotRelative(),

            new SwerveDriveEngage()
        );

        // addCommands(
        //     new SwerveDriveFollowTrajectory(
        //         paths.get("Dock")
        //     ).fieldRelative()
        //     // new (robot.swerve); 
        // );
    
    }

}