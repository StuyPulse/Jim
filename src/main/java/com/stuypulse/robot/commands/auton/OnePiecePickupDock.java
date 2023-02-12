package com.stuypulse.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.arm.routines.ArmIntake;
import com.stuypulse.robot.commands.arm.routines.ArmNeutral;
import com.stuypulse.robot.commands.arm.routines.ArmReady;
import com.stuypulse.robot.commands.arm.routines.ArmScore;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
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

public class OnePiecePickupDock extends SequentialCommandGroup{

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double INTAKE_ACQUIRE_TIME = 0.5;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);

    private HashMap<String, PathPlannerTrajectory> paths;

    public OnePiecePickupDock() {
        paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1.5 Piece + Dock", CONSTRAINTS, CONSTRAINTS),
            "Intake Piece", "Dock"
        );

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
            new IntakeStop()
        );

        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),

            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece"))
                    .robotRelative()
                    .alongWith(new ArmIntake().andThen(new IntakeAcquire()))
        );
        
        addCommands(
            new WaitCommand(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),

            new SwerveDriveFollowTrajectory(
                paths.get("Dock"))
                    .fieldRelative()
                    .alongWith(new ArmNeutral()),

            new SwerveDriveEngage()
        );
    
    }
}
