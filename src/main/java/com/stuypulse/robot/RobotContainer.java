/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.arm.*;
import com.stuypulse.robot.commands.auton.*;
import com.stuypulse.robot.commands.odometry.OdometryReset;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.commands.arm.routines.*;
import com.stuypulse.robot.commands.swerve.*;
import com.stuypulse.robot.subsystems.Manager;
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
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.input.gamepads.Xbox;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);
    
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
        
        CameraServer.startAutomaticCapture();
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
        driver.getTopButton().onTrue(new OdometryReset(new Pose2d(2.94, 0.82, Rotation2d.fromDegrees(0))));
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
