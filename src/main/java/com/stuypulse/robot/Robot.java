/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.TeleopInit;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer robot;
    private Command auto;

    private CommandScheduler scheduler;

    private double disableStartTime;

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        DataLogManager.start();

        scheduler = CommandScheduler.getInstance();
        robot = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        scheduler.run();
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        disableStartTime = Timer.getFPGATimestamp();
        robot.arm.setCoast(true);
        if (Timer.getFPGATimestamp() - disableStartTime >= 6) {
            robot.swerve.setCoast(true);
        }
    }

    @Override
    public void disabledPeriodic() {}

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        robot.arm.setCoast(false);
        robot.swerve.setCoast(false);

        RobotContainer.setCachedAlliance(DriverStation.getAlliance());

        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        robot.arm.setCoast(false);
        robot.swerve.setCoast(false);

        new TeleopInit().schedule();

        RobotContainer.setCachedAlliance(DriverStation.getAlliance());

        if (auto != null) {
            auto.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        robot.arm.setCoast(false);
        
        RobotContainer.setCachedAlliance(DriverStation.getAlliance());

        scheduler.cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
