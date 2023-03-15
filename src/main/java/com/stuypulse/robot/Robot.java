/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.TeleopInit;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer robot;
    private Command auto;

    private CommandScheduler scheduler;

    private enum MatchState {
        AUTO,
        TELEOP,
        TEST,
        DISABLE
    }

    private MatchState state = MatchState.DISABLE;

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        DataLogManager.start();

        scheduler = CommandScheduler.getInstance();
        robot = new RobotContainer();

        state = MatchState.DISABLE;
        SmartDashboard.putString("Match State", state.name());
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
        if (state == MatchState.TELEOP) {
            robot.arm.setCoast(true, true);
        } else {
            robot.arm.setCoast(false, false);
        }

        state = MatchState.DISABLE;
        SmartDashboard.putString("Match State", state.name());
    }

    @Override
    public void disabledPeriodic() {}

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        state = MatchState.AUTO;
        SmartDashboard.putString("Match State", state.name());


        robot.arm.setCoast(false, false);
        robot.arm.setLimp(true, true);
        robot.arm.setTargetState(robot.arm.getState()); // TODO: ArmHold in auton?

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
        state = MatchState.TELEOP;
        SmartDashboard.putString("Match State", state.name());

        robot.arm.setCoast(false, false);
        robot.arm.setLimp(false, false);
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
        state = MatchState.TEST;
        SmartDashboard.putString("Match State", state.name());

        robot.arm.setCoast(false, false);
        
        RobotContainer.setCachedAlliance(DriverStation.getAlliance());

        scheduler.cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
