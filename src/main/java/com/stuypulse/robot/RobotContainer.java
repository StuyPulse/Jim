/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.arm.*;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.odometry.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.wings.*;
import com.stuypulse.robot.commands.intake.*;

import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.arm.*;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.odometry.*;
import com.stuypulse.robot.subsystems.swerve.*;
import com.stuypulse.robot.subsystems.vision.*;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.wings.*;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.util.*;

import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);
    public final Gamepad chooser = new BootlegXbox(Ports.Gamepad.CHOOSER);
    
    // // Subsystem
    public final Intake intake = Intake.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final Vision vision = Vision.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    public final Arm arm = Arm.getInstance();
    public final Plant plant = Plant.getInstance();
    public final Wings wings = Wings.getInstance();
    
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
        
        DriverStation.silenceJoystickConnectionWarning(true);
        // CameraServer.startAutomaticCapture();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        arm.setDefaultCommand(new ArmDrive(operator));
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
        driver.getDPadLeft().onTrue(new WingRetractLeft());
        driver.getDPadUp().onTrue(new WingRetractRight());

        // arm
        driver.getBottomButton().onTrue(new ArmScore().andThen(new IntakeScore()));
        driver.getTopButton().onTrue(new ArmReady());

        // swerve
        driver.getLeftButton().whileTrue(new SwerveDriveToScorePose());
        driver.getLeftTriggerButton().whileTrue(new SwerveDriveSlowDrive(driver));
        // right trigger -> robotrelative override

        driver.getLeftStickButton().onTrue(new RunCommand(() -> {
            var state = new SwerveModuleState(+0.5, new Rotation2d());
            swerve.setModuleStates(state, state, state, state);
        }, swerve));

        driver.getRightStickButton().onTrue(new RunCommand(() -> {
            var state = new SwerveModuleState(-0.5, Rotation2d.fromDegrees(90));
            swerve.setModuleStates(state, state, state, state);
        }, swerve));
    }

    private void configureOperatorBindings() {
        // intaking
        operator.getRightTriggerButton()
            .onTrue(new ArmIntake().andThen(new IntakeAcquire()))
            .onFalse(new IntakeStop())
            .onFalse(new ArmNeutral());

        // outtake
        operator.getLeftTriggerButton()
            .onTrue(new ArmIntake().andThen(new IntakeDeacquire()))
            .onFalse(new IntakeStop())
            .onFalse(new ArmNeutral());

        // ready & score
        operator.getLeftBumper().onTrue(new ArmReady());
        operator.getRightBumper().onTrue(new ArmScore().andThen(new IntakeScore()));

        // set level to score at
        operator.getDPadDown().onTrue(new ManagerSetNodeLevel(NodeLevel.LOW));
        operator.getDPadLeft().onTrue(new ManagerSetNodeLevel(NodeLevel.MID));
        operator.getDPadUp().onTrue(new ManagerSetNodeLevel(NodeLevel.HIGH));
    
        // set game piece
        operator.getLeftButton().onTrue(new ManagerSetGamePiece(GamePiece.CUBE));
        operator.getTopButton().onTrue(new ManagerSetGamePiece(GamePiece.CONE));
        // TODO: CONE_TIP_OUT

        // flip intake side
        operator.getRightButton().onTrue(new ManagerFlipIntakeSide());

        // arm to neutral
        operator.getDPadRight().onTrue(new ArmNeutral());

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
        // autonChooser.addOption("One Piece Dock", new OnePieceDock());
        autonChooser.addOption("Two Piece Dock", new TwoPieceDock());
        autonChooser.addOption("Three Piece", new ThreePiece());
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
