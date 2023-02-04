/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.arm.*;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.MobilityAuton;
import com.stuypulse.robot.commands.auton.OnePiece;
import com.stuypulse.robot.commands.auton.OnePieceDock;
import com.stuypulse.robot.commands.auton.ThreePiece;
import com.stuypulse.robot.commands.auton.ThreePieceDock;
import com.stuypulse.robot.commands.auton.TwoPieceDock;
import com.stuypulse.robot.commands.odometry.OdometryReset;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveToPose;
import com.stuypulse.robot.commands.Wings.*;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.Manager.*;
import com.stuypulse.robot.subsystems.arm.*;
import com.stuypulse.robot.util.*;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.odometry.*;
import com.stuypulse.robot.subsystems.swerve.*;
import com.stuypulse.robot.subsystems.vision.*;
import com.stuypulse.robot.subsystems.plant.*;
import com.stuypulse.robot.subsystems.wings.*;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final IIntake intake = IIntake.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final IVision vision = IVision.getInstance();
    public final IOdometry odometry = IOdometry.getInstance();
    public final IArm arm = IArm.getInstance();
    public final IPlant plant = IPlant.getInstance();
    public final IWings wings = IWings.getInstance();

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
        
        CameraServer.startAutomaticCapture();
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
        driver.getTopButton().onTrue(new OdometryReset());

        driver.getLeftButton().whileTrue(
            new SwerveDriveToPose(new Pose2d(2.25,1.1 , new Rotation2d(Units.degreesToRadians(0) ))));

        driver.getRightButton().whileTrue(new SwerveDriveToPose(new Pose2d(2.25,2.725, new Rotation2d(Units.degreesToRadians(0)))));

        driver.getBottomButton().whileTrue(new SwerveDriveToPose(new Pose2d(2.25,4.4
        ,new Rotation2d(Units.degreesToRadians(0)))));

        
        // driver.getBottomButton().onTrue(new SequentialCommandGroup(
        //                                 new SetPiece(Piece.CONE), 
        //                                 new SetLevel(Level.HIGH), 
        //                                 new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.READY))));
        // driver.getLeftButton().onTrue(new ArmFollowTrajectory(manager.toHome()));
        // driver.getRightButton().onTrue(new SequentialCommandGroup(
        //                                 new SetPiece(Piece.CONE), 
        //                                 new SetLevel(Level.HIGH), 
        //                                 new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
        //                                 new WaitCommand(0.1),
        //                                 new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        // operator.getTopButton().onTrue(new SetPiece(Piece.CONE));
        // operator.getLeftButton().onTrue(new SetPiece(Piece.CUBE));
        // operator.getDPadUp().onTrue(new SetLevel(Level.HIGH));
        // operator.getDPadLeft().onTrue(new SetLevel(Level.MID));
        // operator.getDPadDown().onTrue(new SetLevel(Level.LOW));

        // operator.getLeftBumper().onTrue(new ArmFollowTrajectory(manager.getPath(Side.SAME, Mode.READY)));
        // operator.getRightBumper().onTrue(new ArmFollowTrajectory(manager.append(manager.getPath(Side.SAME, Mode.SCORE), 
        //                                                                         manager.getPath(Side.SAME, Mode.NEUTRAL))));

        // operator.getRightTriggerButton().onTrue(new ArmFollowTrajectory(manager.getPath(Side.SAME, Mode.INTAKE)));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Mobility", new MobilityAuton());
        autonChooser.addOption("One Piece", new OnePiece());
        autonChooser.addOption("One Piece Dock", new OnePieceDock());
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
