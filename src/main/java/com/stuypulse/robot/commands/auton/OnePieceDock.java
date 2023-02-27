package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.plant.PlantEngage;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveAlignThenBalance;
import com.stuypulse.robot.commands.swerve.balance.SwerveDriveBalanceBlay;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceDock extends SequentialCommandGroup {

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double NEUTRAL_WAIT_TIME = 3.0;
    private static final double ENGAGE_TIME = 15;

    private static final PathConstraints BACK_AWAY_CONSTRAINTS = new PathConstraints(0.5, 2);
    private static final PathConstraints BALANCE_CONSTRAINTS = new PathConstraints(0.5, 2);

    public OnePieceDock() {

        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1 Piece + Dock", BACK_AWAY_CONSTRAINTS, BALANCE_CONSTRAINTS),
            "Back Away", "Dock"
        );


        // initial setup
        addCommands(
            new LEDSet(LEDColor.RAINBOW),
            new ManagerSetNodeLevel(NodeLevel.HIGH),
            new ManagerSetGamePiece(GamePiece.CONE_TIP_UP),
            new ManagerSetScoreSide(ScoreSide.BACK)
        );

        // score first piece
        addCommands(
            new LEDSet(LEDColor.YELLOW.pulse()),
            new ArmReady(),
            new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new LEDSet(LEDColor.RAINBOW)
        );

        // dock and engage
        addCommands(
            new LEDSet(LEDColor.GREEN),
            new SwerveDriveFollowTrajectory(
                PathPlanner.loadPath("1 Piece + Dock", CONSTRAINTS))
                    .robotRelative()
                    .addEvent("ArmNeutral", new ArmNeutral())
                    .withEvents(),

            
            new SwerveDriveBalanceWithPlant().withTimeout(ENGAGE_TIME),
            new SwerveDriveFollowTrajectory(paths.get("Back Away")),
            new ArmNeutral(),
            new SwerveDriveFollowTrajectory(paths.get("Dock")),
            new SwerveDriveBalanceBlay().withMaxSpeed(BALANCE_CONSTRAINTS.maxVelocity).withTimeout(ENGAGE_TIME),
            new PlantEngage()
            
        );
    
    }

}