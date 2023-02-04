package com.stuypulse.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.commands.intake.IntakeCube;
import com.stuypulse.robot.commands.intake.OuttakeCube;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceDock extends SequentialCommandGroup {

    private HashMap<String, PathPlannerTrajectory> paths;
    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    public TwoPieceDock() {

        paths = FollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece + Dock", Motion.CONSTRAINTS, Motion.CONSTRAINTS),

            "Intake Piece", "Score Piece", "Dock"
        );

        // score held piece, then drive to first game piece and intake
        addCommands(
            new ArmFollowTrajectory(),
            new OuttakeCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),

            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),

            new FollowTrajectory(
                paths.get("Intake Piece")
            ).robotRelative(),
            new IntakeCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)
        );
        
        // drive to grid and score game piece
        addCommands(
            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),

            new FollowTrajectory(
                paths.get("Score Piece")
            ).fieldRelative(),
            new ArmFollowTrajectory(),
            new OuttakeCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // drive to charging station and dock
        addCommands(
            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),

            new FollowTrajectory(
                paths.get("Dock")
            ).fieldRelative()

            // new BasicGyroEngage()
        );
                
    }

}