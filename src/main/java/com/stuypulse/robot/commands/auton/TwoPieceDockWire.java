package com.stuypulse.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.arm.routines.ArmNeutral;
import com.stuypulse.robot.commands.arm.routines.ArmReady;
import com.stuypulse.robot.commands.arm.routines.ArmScore;
import com.stuypulse.robot.commands.intake.IntakeAcquireCube;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCone;
import com.stuypulse.robot.commands.intake.IntakeDeacquireCube;
import com.stuypulse.robot.commands.intake.IntakeScore;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.ManagerSetGamePiece;
import com.stuypulse.robot.commands.manager.ManagerSetNodeLevel;
import com.stuypulse.robot.commands.manager.ManagerSetScoreSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveEngage;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.subsystems.LEDController;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceDockWire extends SequentialCommandGroup {
    private HashMap <String , PathPlannerTrajectory> paths;
    private static final double INTAKE_ACQUIRE_TIME = 0.2;
    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double PARTNER_WAIT_TIME = 0.0;

    private static final PathConstraints CONSTRAINTS = new PathConstraints( 2.5, 1);

    public TwoPieceDockWire(){

        paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("2 Piece + Dock Wire", CONSTRAINTS, CONSTRAINTS),
        "Intake Piece" , "Score Piece", "Dock"
        );

        addCommands(
            new ManagerSetGamePiece(GamePiece.CONE),
            new ManagerSetScoreSide(ScoreSide.SAME),
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ArmReady(),
            new IntakeScore(),
            new ArmScore(), 
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new ArmNeutral()
        );

        // score held piece, then drive to first game piece and intake
        addCommands(
            new IntakeDeacquireCone(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new WaitCommand(PARTNER_WAIT_TIME),
            
            new LEDSet(LEDColor.PURPLE),

            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece")
            ).robotRelative(),
            new IntakeAcquireCube(),
            new WaitCommand(INTAKE_ACQUIRE_TIME)
        );

        // drive to grid and score game piece

        addCommands(
            new LEDSet(LEDColor.PURPLE),

            new SwerveDriveFollowTrajectory(
                paths.get("Score Piece")
                ).fieldRelative(),
            new IntakeDeacquireCube(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        addCommands(
            new SwerveDriveFollowTrajectory(
                paths.get("Dock")
            ).fieldRelative(),

            new SwerveDriveEngage()
        );
    }
}
