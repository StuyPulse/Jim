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
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.util.LEDColor;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPiecePickupDock extends SequentialCommandGroup {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);
    private HashMap<String, PathPlannerTrajectory> paths;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double INTAKE_ACQUIRE_TIME = 1.0;
    
    public TwoPiecePickupDock() {

        paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2.5 Piece + Dock", CONSTRAINTS, CONSTRAINTS),
            "Intake One", "Score One", "Intake Two", "Dock"
        );

        addCommands(

            // Score 1 piece
            new IntakeDeacquireCone(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),
            // Drive to ball
            new SwerveDriveFollowTrajectory(
                paths.get("Intake One")
            ).robotRelative(),

            new IntakeAcquireCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)     
            
        );

        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Score One")
            ).fieldRelative(),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        addCommands (
            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Two")
            ).fieldRelative(),
            new IntakeAcquireCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)
        );

        addCommands( 
            new SwerveDriveFollowTrajectory(
                paths.get("Dock")
            ).fieldRelative(),
            new SwerveDriveEngage()
        );
    }
}
