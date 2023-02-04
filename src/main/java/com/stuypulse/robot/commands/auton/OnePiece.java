package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.commands.intake.OuttakeCube;
import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiece extends SequentialCommandGroup {
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(5, 3);

    public OnePiece() {
        addCommands(
            new ArmFollowTrajectory(),
            new OuttakeCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new FollowTrajectory(
                PathPlanner.loadPath("1 Piece + Mobility", CONSTRAINTS)
            ).robotRelative()
        );
    }
    
}