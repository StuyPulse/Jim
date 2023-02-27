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
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.LEDColor;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiecePickupDock extends SequentialCommandGroup{

    private static final double INTAKE_DEACQUIRE_TIME = 1.0;
    private static final double INTAKE_ACQUIRE_TIME = 3.5;
    private static final double ENGAGE_TIME = 10.0;

    private static final PathConstraints INTAKE_PIECE = new PathConstraints(3, 2);
    private static final PathConstraints DOCK = new PathConstraints(2, 2);

    public OnePiecePickupDock() {
        var paths = SwerveDriveFollowTrajectory.getSeparatedPaths(
            PathPlanner.loadPathGroup("1.5 Piece + Dock", INTAKE_PIECE, DOCK),
            "Intake Piece", "Dock" 
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
            // new ArmScore(),
            new IntakeScore(),
            new WaitCommand(INTAKE_DEACQUIRE_TIME),
            new IntakeStop(),
            new LEDSet(LEDColor.RAINBOW)
            new WaitCommand(INTAKE_DEACQUIRE_TIME)
        );

        // intake second piece
        addCommands(
            new ManagerSetGamePiece(GamePiece.CUBE),

            new LEDSet(LEDColor.GREEN),
            new SwerveDriveFollowTrajectory(
                paths.get("Intake Piece"))
                    .robotRelative()
                    .alongWith(new IntakeAcquire().andThen(new ArmIntake())),

            new LEDSet(LEDColor.YELLOW),
            new IntakeWaitForPiece().withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop(),
            new LEDSet(LEDColor.RAINBOW)
            new IntakeAcquire().withTimeout(0.5),
            new IntakeStop()
        );
        
        // dock and engage
        addCommands(
            new LEDSet(LEDColor.GREEN),
            new SwerveDriveFollowTrajectory(
                paths.get("Dock"))
                    .fieldRelative()
                    .addEvent("ArmNeutral", new ArmNeutral())
                    .withEvents(),
                    
            
            new SwerveDriveBalanceWithPlant().withTimeout(ENGAGE_TIME),
            new SwerveDriveAlignThenBalance().withTimeout(ENGAGE_TIME),
            new PlantEngage()
            
        );
    
    }
}
