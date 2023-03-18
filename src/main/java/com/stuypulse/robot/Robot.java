/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.AutonInit;
import com.stuypulse.robot.commands.TeleopInit;
import com.stuypulse.robot.commands.TestInit;

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

    private static MatchState kMatchState = MatchState.DISABLE;

    private static void setMatchState(MatchState state) {
        kMatchState = state;
        SmartDashboard.putString("Match State", kMatchState.name());
    }


    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        DataLogManager.start();

        scheduler = CommandScheduler.getInstance();
        robot = new RobotContainer();

        setMatchState(MatchState.DISABLE);
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
        if (kMatchState == MatchState.AUTO) {
            robot.arm.enableShoulderBrakeMode();
            robot.arm.enableWristBrakeMode();
        } else {
            robot.arm.enableShoulderCoastMode();
            robot.arm.enableWristCoastMode();
        }

        setMatchState(MatchState.DISABLE);
    }

    @Override
    public void disabledPeriodic() {}

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        RobotContainer.setCachedAlliance(DriverStation.getAlliance());

        setMatchState(MatchState.AUTO);
        new AutonInit().schedule();

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
        RobotContainer.setCachedAlliance(DriverStation.getAlliance());

        setMatchState(MatchState.TELEOP);
        new TeleopInit().schedule();

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
        RobotContainer.setCachedAlliance(DriverStation.getAlliance());

        setMatchState(MatchState.TELEOP);
        new TestInit().schedule();

        robot.arm.enableShoulderBrakeMode();
        robot.arm.enableWristBrakeMode();

        scheduler.cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
