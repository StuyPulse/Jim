    /************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.*;
import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.robot.commands.*;
import com.stuypulse.robot.commands.arm.*;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.auton.battlecry.BCThreePieceBumpBlue;
import com.stuypulse.robot.commands.auton.battlecry.BCThreePieceBumpRed;
import com.stuypulse.robot.commands.auton.battlecry.BCTwoPieceDockBumpBlue;
import com.stuypulse.robot.commands.auton.battlecry.BCTwoPieceDockBumpRed;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.manager.*;
import com.stuypulse.robot.commands.odometry.*;
import com.stuypulse.robot.commands.plant.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.commands.swerve.balance.*;
import com.stuypulse.robot.commands.wing.*;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.subsystems.arm.*;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.odometry.*;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.swerve.*;
import com.stuypulse.robot.subsystems.vision.*;
import com.stuypulse.robot.subsystems.wing.*;
import com.stuypulse.robot.util.*;
import com.stuypulse.robot.util.BootlegXbox;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);

    // Subsystem
   
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Vision vision = Vision.getInstance();
    public final Odometry odometry = Odometry.getInstance();
    public final Arm arm = Arm.getInstance();
    public final Plant plant = Plant.getInstance();
    public final Wing wing = Wing.getInstance();

    public final Manager manager = Manager.getInstance();
    public final LEDController leds = LEDController.getInstance();
    public final Pump pump = Pump.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    private static Alliance cachedAlliance;

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        LiveWindow.disableAllTelemetry();
        // DriverStation.silenceJoystickConnectionWarning(true);
        // CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
        if (Robot.isReal())
            CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 80, 60, 30);

        SmartDashboard.putData("Gamepads/Driver", driver);
        SmartDashboard.putData("Gamepads/Operator", operator);
        // SmartDashboard.putData("Gamepads/Chooser", chooser);
        
       

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

        new Trigger(new SmartBoolean("BOOM/ACQUIRE", false)::get)
            .onTrue(new IntakeAcquire())
            .onFalse(new IntakeStop());

        new Trigger(new SmartBoolean("BOOM/DEACQUIRE", false)::get)
            .onTrue(new IntakeDeacquire())
            .onFalse(new IntakeStop());
    }

    private void configureDriverBindings() {
        // arm
        driver.getLeftTriggerButton()
            .whileTrue(new RobotScore());
        driver.getBottomButton()
            .whileTrue(new RobotScore());
        driver.getRightTriggerButton()
            .whileTrue(new RobotRelease())
            .onFalse(new WaitCommand(0.5).andThen(new IntakeStop()));

        driver.getTopButton()
            .onTrue(new ManagerValidateState())
            .onTrue(new ManagerChooseScoreNode())
            .whileTrue(new RobotAlignThenScoreCubes());

        // swerve
        //driver.getLeftButton().whileTrue(new SwerveDriveAlignThenBalance());
       
        // left bumper -> robot relative

        // odometry
        driver.getDPadUp().onTrue(new OdometryRealign(Rotation2d.fromDegrees(180)));
        driver.getDPadLeft().onTrue(new OdometryRealign(Rotation2d.fromDegrees(-90)));
        driver.getDPadDown().onTrue(new OdometryRealign(Rotation2d.fromDegrees(0)));
        driver.getDPadRight().onTrue(new OdometryRealign(Rotation2d.fromDegrees(90)));

        // plant
        // driver.getRightButton().onTrue(new PlantEngage());
        driver.getRightButton().toggleOnTrue(new SwerveDriveAndPoint(driver, new Translation2d()));

        //driver.getRightBumper().onTrue(new PlantDisengage());

        new Trigger(intake::hasGamePiece)
            .and(DriverStation::isTeleop)
            .debounce(0.5, DebounceType.kFalling)
            .onTrue(new LEDSet(LEDColor.RED))
            .onTrue(new InstantCommand(() -> driver.setRumble(0.5)))
            .onFalse(new InstantCommand(() -> driver.setRumble(0.0)));
    }

    private void configureOperatorBindings() {
        // manual control
        new Trigger(() -> (operator.getLeftStick().magnitude() + operator.getRightStick().magnitude()) > Settings.Operator.DEADBAND.get())
            .onTrue(new ArmVoltageDrive(operator));

        // wing
        operator.getSelectButton().onTrue(new WingExtend());
        operator.getStartButton().onTrue(new WingRetract());

        // intaking
        operator.getRightTriggerButton()
            .whileTrue(new ArmIntake())
            .onTrue(new IntakeAcquire())
            .onFalse(new IntakeStop())
            .onFalse(new ArmStow());

        // intaking from HP
        operator.getLeftTriggerButton()
            .whileTrue(new ArmIntakeHP())
            .onTrue(new IntakeAcquire())
            .onFalse(new IntakeStop())
            .onFalse(new ArmStow());

        // ready & score
        operator.getLeftBumper()
            .onTrue(new ManagerValidateState()
                .andThen(new IntakeAcquireColyi())
                .andThen(new WaitCommand(0.2)
                    .until(() -> intake.hasGamePiece()))
                .andThen(new IntakeStop()))
            .whileTrue(
                new LEDSet(LEDColor.RED)
                    .andThen(new ManagerValidateState())
                    .andThen(new ArmReady()))
            .onFalse(new IntakeAcquireColyi()
                .andThen(new WaitCommand(0.2)
                    .until(() -> intake.hasGamePiece()))
                .andThen(new IntakeStop()));


        operator.getRightButton()
            .onTrue(new IntakeScore())
            .onFalse(new IntakeStop());

        // set level to score at
        operator.getDPadDown().onTrue(new ManagerSetNodeLevel(NodeLevel.LOW));
        operator.getDPadLeft().onTrue(new ManagerSetNodeLevel(NodeLevel.MID));
        operator.getDPadUp().onTrue(new ManagerSetNodeLevel(NodeLevel.HIGH));

        // set game piece
        operator.getLeftButton()
            .onTrue(new ManagerSetGamePiece(GamePiece.CUBE));

        operator.getTopButton()
            .onTrue(new ManagerSetGamePiece(GamePiece.CONE_TIP_IN));

        operator.getBottomButton()
            .onTrue(new ManagerSetGamePiece(GamePiece.CONE_TIP_OUT));

        operator.getRightBumper()
            .onTrue(arm.runOnce(arm::enableLimp))
            .onFalse(arm.runOnce(arm::disableLimp));

        // arm to neutral
        operator.getDPadRight().onTrue(new IntakeAcquire())
            .onFalse(new IntakeStop());
        
        new Trigger(intake::hasGamePiece)
            .and(DriverStation::isTeleop)
            .debounce(0.5, DebounceType.kFalling)
            .onTrue(new InstantCommand(() -> operator.setRumble(0.5)))
            .onFalse(new InstantCommand(() -> operator.setRumble(0.0)));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothing());
        // autonChooser.addOption("Mobility", new Mobility());

        // // One Piece

        // // autonChooser.addOption("One Piece Dock", new OnePieceDock());
        // autonChooser.addOption("One Piece Mobility Dock", new OnePieceMobilityDock());

        // // One Piece Dock

        // // autonChooser.addOption("1.5 Piece Dock", new OnePiecePickupDock());
        // // autonChooser.addOption("1.5 Piece Dock Bump", new OnePiecePickupDockBump());

        // // Two Piece

        // autonChooser.addOption("Two Piece", new TwoPiece());
        // autonChooser.addOption("Two Piece Bump", new TwoPieceBump());
        // autonChooser.addOption("Two Piece Dock Red", new TwoPieceDockRed());
        // autonChooser.setDefaultOption("Two Piece Dock Blue", new TwoPieceDockBlue());
        // // autonChooser.addOption("Two Piece Dock Bump Removed Blue", new BCTwoPieceDockBumpBlue());
        // // autonChooser.addOption("Two Piece Dock Bump Removed Red", new BCTwoPieceDockBumpRed());

        // // Three Piece

        // autonChooser.addOption("Three Piece", new ThreePiece()); // basically blue
        // autonChooser.addOption("Three Piece Red", new ThreePieceRed());
        // autonChooser.addOption("Three Piece Blue", new ThreePieceBlue());
        // // autonChooser.addOption("Three Piece Bump Removed Blue", new BCThreePieceBumpBlue());
        // // autonChooser.addOption("Three Piece Bump Removed Red", new BCThreePieceBumpRed());
        // autonChooser.addOption("Three Piece Bump", new ThreePieceBump());

        // SmartDashboard.putData("Autonomous", autonChooser);
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
