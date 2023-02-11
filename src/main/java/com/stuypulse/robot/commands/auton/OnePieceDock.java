package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCone;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCube;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
// import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveEngage;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceDock extends SequentialCommandGroup {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    public OnePieceDock() {
        addCommands(
            new IntakeDeacquireCone(),
            new LEDSet(LEDController.getInstance(), LEDColor.PURPLE),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("1 Piece + Mobility + Dock", CONSTRAINTS)
            ).robotRelative(),

            new SwerveDriveEngage()
        );
    
    }

}