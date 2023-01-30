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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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

        driver.getBottomButton().onTrue(new SequentialCommandGroup(
                                        new SetPiece(Piece.CONE), 
                                        new SetLevel(Level.HIGH), 
                                        new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.READY))));
        driver.getLeftButton().onTrue(new ArmFollowTrajectory(manager.toHome()));
        driver.getRightButton().onTrue(new SequentialCommandGroup(
                                        new SetPiece(Piece.CONE), 
                                        new SetLevel(Level.HIGH), 
                                        new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                        new WaitCommand(0.1),
                                        new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
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
        autonChooser.addOption("High Cone Ready", new SequentialCommandGroup(
                                            new SetPiece(Piece.CONE), 
                                            new SetLevel(Level.HIGH), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("High Cone Score", new SequentialCommandGroup(
                                            new SetPiece(Piece.CONE), 
                                            new SetLevel(Level.HIGH), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("Mid Cone Ready", new SequentialCommandGroup(
                                            new SetPiece(Piece.CONE), 
                                            new SetLevel(Level.MID), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("Mid Cone Score", new SequentialCommandGroup(
                                            new SetPiece(Piece.CONE), 
                                            new SetLevel(Level.MID), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("Low Cone Ready", new SequentialCommandGroup(
                                            new SetPiece(Piece.CONE), 
                                            new SetLevel(Level.LOW), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("Low Cone Score", new SequentialCommandGroup(
                                            new SetPiece(Piece.CONE), 
                                            new SetLevel(Level.LOW), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("High Cube Ready", new SequentialCommandGroup(
                                            new SetPiece(Piece.CUBE), 
                                            new SetLevel(Level.HIGH), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("High Cube Score", new SequentialCommandGroup(
                                            new SetPiece(Piece.CUBE), 
                                            new SetLevel(Level.HIGH), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("Mid Cube Ready", new SequentialCommandGroup(
                                            new SetPiece(Piece.CUBE), 
                                            new SetLevel(Level.MID), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("Mid Cube Score", new SequentialCommandGroup(
                                            new SetPiece(Piece.CUBE), 
                                            new SetLevel(Level.MID), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("Low Cube Ready", new SequentialCommandGroup(
                                            new SetPiece(Piece.CUBE), 
                                            new SetLevel(Level.LOW), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));
        autonChooser.addOption("Low Cube Score", new SequentialCommandGroup(
                                            new SetPiece(Piece.CUBE), 
                                            new SetLevel(Level.LOW), 
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.SCORE)),
                                            new WaitCommand(0.1),
                                            new ArmFollowTrajectory(manager.getTrajectory(Side.SAME, Mode.NEUTRAL))));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
