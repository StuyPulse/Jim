/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;

public class ArmOuttake extends ArmRoutine {

    public ArmOuttake() {
        super(Manager.getInstance()::getOuttakeTrajectory);
    }
}
