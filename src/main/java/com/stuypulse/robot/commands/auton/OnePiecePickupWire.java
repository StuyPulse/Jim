package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.ArmIntake;
import com.stuypulse.robot.commands.arm.routines.ArmNeutral;
import com.stuypulse.robot.commands.arm.routines.ArmReady;
import com.stuypulse.robot.commands.arm.routines.ArmScore;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeScore;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.manager.ManagerSetGamePiece;
import com.stuypulse.robot.commands.manager.ManagerSetIntakeSide;
import com.stuypulse.robot.commands.manager.ManagerSetNodeLevel;
import com.stuypulse.robot.commands.manager.ManagerSetScoreSide;
import com.stuypulse.robot.commands.swerve.SwerveDriveFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;
import com.stuypulse.robot.subsystems.Manager.NodeLevel;
import com.stuypulse.robot.subsystems.Manager.ScoreSide;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiecePickupWire extends SequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double INTAKE_ACQUIRE_TIME = 0.5;

    private static final PathConstraints CONSTRAINTS = new PathConstraints(2, 2);

    public OnePiecePickupWire(){
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
                PathPlanner.loadPath("1.5 Piece + Wire", CONSTRAINTS))
                    .robotRelative()
                    .alongWith(new ArmIntake().andThen(new IntakeAcquire()))
        );
        
        addCommands(
            new WaitCommand(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new ArmNeutral()
        );

    }

}
