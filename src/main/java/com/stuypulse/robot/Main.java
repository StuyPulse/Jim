/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.util.StopWatch;
import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {

    private Main() {}

    public static void main(String... args) {
        StopWatch.setDefaultEngine(StopWatch.kFPGAEngine);
        RobotBase.startRobot(Robot::new);
    }
}
