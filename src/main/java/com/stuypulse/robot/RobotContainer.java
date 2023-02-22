/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import java.util.function.Supplier;

import com.stuypulse.robot.commands.arm.*;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.odometry.*;
import com.stuypulse.robot.commands.plant.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.*;
import com.stuypulse.robot.commands.wing.*;
import com.stuypulse.robot.commands.intake.*;

import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.arm.*;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.odometry.*;
import com.stuypulse.robot.subsystems.swerve.*;
import com.stuypulse.robot.subsystems.vision.*;
import com.stuypulse.robot.subsystems.wing.*;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.constants.ArmFields;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.*;

import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.*;

import edu.wpi.first.cameraserver.CameraServer;
import 
edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);
    public final Gamepad chooser = new BootlegXbox(Ports.Gamepad.CHOOSER);
    
    // // Subsystem
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Vision vision = Vision.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    public final Arm arm = Arm.getInstance();
    public final Plant plant = Plant.getInstance();
    public final Wing wing = Wing.getInstance();
    
    public final Manager manager = Manager.getInstance();
    public final LEDController leds = LEDController.getInstance();
    public final Pump pump = new Pump();
  
    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    private static Alliance cachedAlliance;

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        LiveWindow.disableAllTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);
        CameraServer.startAutomaticCapture();

        ArmFields.load();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureOperatorBindings();
        configureDriverBindings();
        configureChooserBindings();
    }

    private void configureDriverBindings() {
        // wing
        driver.getSelectButton().onTrue(new WingToggle());

        // arm
        driver.getBottomButton()
            .onTrue(new ArmScore().alongWith(new IntakeScore()))
            .onFalse(new IntakeStop());
        driver.getTopButton().onTrue(new ArmReady());

        driver.getRightButton().onTrue(new ManagerFlipScoreSide());

        // swerve
        driver.getLeftButton()
            .whileTrue(new ManagerChooseScoreSide().andThen(new SwerveDriveToScorePose()));
        driver.getLeftTriggerButton().whileTrue(new SwerveDriveAlignThenBalance());
        driver.getDPadUp().onTrue(new OdometryRealign(Rotation2d.fromDegrees(180)));
        driver.getDPadDown().onTrue(new OdometryRealign(Rotation2d.fromDegrees(0)));
        // right trigger -> robotrelative override

        // plant
        driver.getLeftBumper().onTrue(new PlantEngage());
        driver.getRightBumper().onTrue(new PlantDisengage());

    }

    private void configureOperatorBindings() {
        // manual control
        new Trigger(() -> (operator.getLeftStick().magnitude() + operator.getRightStick().magnitude()) > Settings.Operator.DEADBAND.get()).onTrue(new ArmDrive(operator));
        
        // intaking
        operator.getRightTriggerButton()
            .whileTrue(new ArmIntake().alongWith(new IntakeAcquire()))
            .onFalse(new IntakeStop())
            .onFalse(new ArmNeutral());

        // outtake
        operator.getLeftTriggerButton()
            .whileTrue(new ArmOuttake().alongWith(new IntakeDeacquire()))
            .onFalse(new IntakeStop())
            .onFalse(new ArmNeutral());

        // ready & score
        operator.getLeftBumper()
            .whileTrue(new ArmReady())
            .onTrue(new ManagerValidateState().andThen(new ManagerChooseScoreSide()));

        operator.getRightBumper()
            .whileTrue(new ArmScore().alongWith(new IntakeScore()))
            .onFalse(new IntakeStop());

        // set level to score at
        operator.getDPadDown().onTrue(new ManagerSetNodeLevel(NodeLevel.LOW));
        operator.getDPadLeft().onTrue(new ManagerSetNodeLevel(NodeLevel.MID));
        operator.getDPadUp().onTrue(new ManagerSetNodeLevel(NodeLevel.HIGH));
    
        // set game piece
        operator.getLeftButton()
            .onTrue(new ManagerSetGamePiece(GamePiece.CUBE))
            .onTrue(new ManagerSetScoreSide(ScoreSide.OPPOSITE));

        operator.getTopButton()
            .onTrue(new ManagerSetGamePiece(GamePiece.CONE_TIP_IN))
            .onTrue(new ManagerSetScoreSide(ScoreSide.OPPOSITE));

        operator.getBottomButton()
            .onTrue(new ManagerSetGamePiece(GamePiece.CONE_TIP_OUT))
            .onTrue(new ManagerSetScoreSide(ScoreSide.SAME));

        // flip intake side
        operator.getRightButton().onTrue(new ManagerFlipIntakeSide());

        // arm to neutral
        operator.getDPadRight().onTrue(new ManagerFlipScoreSide());

        // manual overrides
        operator.getSelectButton().onTrue(arm.runOnce(arm::enableFeedback));
        operator.getStartButton().onTrue(arm.runOnce(arm::disableFeedback));

    }

    private void configureChooserBindings() {
        chooser.getDPadLeft().onTrue(new ManagerSetGridSection(Direction.LEFT));
        chooser.getDPadUp().onTrue(new ManagerSetGridSection(Direction.CENTER));
        chooser.getDPadRight().onTrue(new ManagerSetGridSection(Direction.RIGHT));
        
        chooser.getLeftButton().onTrue(new ManagerSetGridColumn(Direction.LEFT));
        chooser.getTopButton().onTrue(new ManagerSetGridColumn(Direction.CENTER));
        chooser.getRightButton().onTrue(new ManagerSetGridColumn(Direction.RIGHT));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Mobility", new MobilityAuton());
        autonChooser.addOption("One Piece", new OnePiece());
        autonChooser.addOption("One Piece Wire", new OnePiecePickupWire());
        autonChooser.addOption("One Piece + Dock", new OnePieceDock());
        autonChooser.addOption("1.5 Piece Dock", new OnePiecePickupDock());
        autonChooser.addOption("Two Piece", new TwoPiece());
        autonChooser.addOption("Two Piece Wire", new TwoPieceWire());
        autonChooser.addOption("Two Piece Dock", new TwoPieceDock());
        autonChooser.addOption("2.5 Piece", new TwoPiecePickup());
        autonChooser.addOption("2.5 Piece Dock", new TwoPiecePickupDock());
        autonChooser.addOption("Three Piece", new ThreePiece());
        autonChooser.addOption("Three Piece Wire", new ThreePieceWire());
        autonChooser.addOption("Three Piece Dock", new ThreePieceDock());

        
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static void setCachedAlliance(Alliance alliance) {
        cachedAlliance = alliance;
    }

    public static Alliance getCachedAlliance() {
        return cachedAlliance;
    }
}
