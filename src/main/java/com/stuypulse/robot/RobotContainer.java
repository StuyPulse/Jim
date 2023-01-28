/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.arm.*;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.manager.SetLevel;
import com.stuypulse.robot.commands.manager.SetPiece;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.ArmTrajectories.OppositeSide.Mid;
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
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // // Subsystem
    // public final IIntake intake = IIntake.getInstance();
    // public final SwerveDrive swerve = SwerveDrive.getInstance();
    // public final IVision vision = IVision.getInstance();
    // public final IOdometry odometry = IOdometry.getInstance();
    public final IArm arm = IArm.getInstance();
    public final IPlant plant = IPlant.getInstance();
    public final IWings wings = IWings.getInstance();
    
    public final Manager manager = Manager.getInstance();
    public final LEDController leds = new LEDController(this);
    public final Pump pump = new Pump();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

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

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        operator.getTopButton().onTrue(new SetPiece(Piece.CONE));
        operator.getLeftButton().onTrue(new SetPiece(Piece.CUBE));
        operator.getDPadUp().onTrue(new SetLevel(Level.HIGH));
        operator.getDPadLeft().onTrue(new SetLevel(Level.MID));
        operator.getDPadDown().onTrue(new SetLevel(Level.LOW));

        operator.getLeftBumper().onTrue(new ArmFollowTrajectory(manager.getPath(Side.SAME, Mode.READY)));
        operator.getRightBumper().onTrue(new ArmFollowTrajectory(manager.append(manager.getPath(Side.SAME, Mode.SCORE), 
                                                                                manager.getPath(Side.SAME, Mode.NEUTRAL))));

        operator.getRightTriggerButton().onTrue(new ArmFollowTrajectory(manager.getPath(Side.SAME, Mode.INTAKE)));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
