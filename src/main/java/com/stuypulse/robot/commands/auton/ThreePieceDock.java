package com.stuypulse.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.IntakeAcquireCube;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCube;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceDock extends SequentialCommandGroup {

    private HashMap<String, PathPlannerTrajectory> paths;
    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(5, 3);

    public ThreePieceDock() {


        // load paths into hashmap
        paths = FollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("3 Piece + Dock", CONSTRAINTS, CONSTRAINTS),

            "Intake Piece", "Score Piece", "Intake Piece Two", "Score Piece Two", "Dock"
        );

        // Drive to first piece and intake
        addCommands(
            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),
            
            // new ArmFollowTrajectory(),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            
            new FollowTrajectory(
                paths.get("Intake Piece")
            ).robotRelative(),
            new IntakeAcquireCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)
        );

        // Drive to grid and score one piece
        addCommands(
            new LEDSet(LEDController.getInstance(), LEDColor.BLUE),

            new FollowTrajectory(
                paths.get("Score Piece")
            ).fieldRelative(),
            // new ArmFollowTrajectory(),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // Drive to second game piece and intake 
        addCommands(

            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),

            new FollowTrajectory(
                paths.get("Intake Piece Two")
            ).fieldRelative(),
            new IntakeAcquireCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)

        );

        // Drive to grid and score second piece

        addCommands(

            new LEDSet(LEDController.getInstance(), LEDColor.GREEN),

            new FollowTrajectory(
                paths.get("Score Piece Two")
            ).fieldRelative(),
            // new ArmFollowTrajectory(),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)

        );

        // Drive to charging station and dock
        addCommands(

            new LEDSet(LEDController.getInstance(), LEDColor.BLUE),

            new FollowTrajectory(
                paths.get("Dock")
            ).fieldRelative()

            // new BasicGyroEngage()
        );
                
    }

}