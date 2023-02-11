package com.stuypulse.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.IntakeAcquireCube;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCone;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCube;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.swerve.SwerveDriveEngage;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.robot.subsystems.LEDController;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiecePickupDock extends SequentialCommandGroup{
    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);
    private HashMap<String, PathPlannerTrajectory> paths;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    public OnePiecePickupDock() {
        paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1.5 Piece + Dock", CONSTRAINTS, CONSTRAINTS),
            "Intake Piece", "Dock"
        );

        addCommands(
            // new ArmFollowTrajectory(),
            new IntakeDeacquireCone(),
            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece")
            ).robotRelative()
        );

        addCommands(
            new IntakeAcquireCube(),

            new SwerveDriveFollowTrajectory(
                paths.get("Dock")
            ),

            new SwerveDriveEngage()
        );
    
    }
}
