package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.OuttakeCube;
import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceDock extends SequentialCommandGroup {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(5, 3);
    private HashMap<String, PathPlannerTrajectory> paths;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    public OnePieceDock() {
        paths = FollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1 Piece + Dock", CONSTRAINTS, CONSTRAINTS),

            "Mobility"
        );

        addCommands(
            // new ArmFollowTrajectory(null),
            new OuttakeCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new FollowTrajectory(
                paths.get("Mobility")
            ).robotRelative()
        );

        addCommands(
            new FollowTrajectory(
                paths.get("Dock")
            ).fieldRelative()
            // new BasicGyroEngage(robot.swerve); 
        );
    
    }

}