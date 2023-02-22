package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MobilityAuton extends SequentialCommandGroup {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);
    
    public MobilityAuton() {
        addCommands(
            new LEDSet(LEDColor.GREEN),
            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("Mobility", CONSTRAINTS)
            ).robotRelative()
        );
    }
    
}