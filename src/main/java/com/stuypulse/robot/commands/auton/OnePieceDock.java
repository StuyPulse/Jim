package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.ArmNeutral;
import com.stuypulse.robot.commands.arm.routines.ArmReady;
import com.stuypulse.robot.commands.arm.routines.ArmScore;
import com.stuypulse.robot.commands.intake.IntakeScore;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.swerve.SwerveDriveEngage;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceDock extends SequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);

    public OnePieceDock() {
        addCommands(
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE),
            new ManagerSetIntakeSide(IntakeSide.FRONT),
            new ManagerSetScoreSide(ScoreSide.OPPOSITE)
            
            // new ManagerSetGridSection()
            // new ManagerSetGridColumn()
        );

        addCommands(
            new ArmReady(),
            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

        addCommands(
            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("1 Piece + Mobility + Dock", CONSTRAINTS)
            ).robotRelative(),

            new SwerveDriveEngage()
        );
    
    }

}