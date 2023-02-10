package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCube;
import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceDockWire extends SequentialCommandGroup {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(5, 3);
    private HashMap<String, PathPlannerTrajectory> paths;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    public OnePieceDockWire() {
        paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1 Piece + Dock Wire", CONSTRAINTS, CONSTRAINTS),

            "Mobility", "Dock"
        );

        addCommands(
            // new ArmFollowTrajectory(null),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new SwerveDriveFollowTrajectory(
                paths.get("Mobility")
            ).robotRelative()
        );

        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Dock")
            ).fieldRelative()
            // new BasicGyroEngage(robot.swerve); 
        );
    
    }

}