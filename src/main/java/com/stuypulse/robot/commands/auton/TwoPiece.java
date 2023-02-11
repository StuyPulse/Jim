package com.stuypulse.robot.commands.auton;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.IntakeAcquireCube;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCone;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCube;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.constants.Ports.LEDController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPiece extends SequentialCommandGroup{
    private HashMap<String, PathPlannerTrajectory> paths;
    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(5, 3);

    public TwoPiece () {

        paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece", CONSTRAINTS, CONSTRAINTS),

            "Intake Piece", "Score Piece"
        );
        // score held piece, then drive to first game piece and intake
        addCommands(
            // new ArmFollowTrajectory(),
            new IntakeDeacquireCone(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),

            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),

            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece")
            ).robotRelative(),
            new IntakeAcquireCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)
        );
        
        // drive to grid and score game piece
        addCommands(
            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),

            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece")
            ).fieldRelative(),
            // new ArmFollowTrajectory(),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );       

    }

}