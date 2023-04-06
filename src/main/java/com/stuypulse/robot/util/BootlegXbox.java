/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.input.gamepads.Xbox;

/*
TODO:
add .flipped() to Xbox.java in stuylib

*/
public class BootlegXbox extends Xbox {

    public BootlegXbox(int port) {
        super(port);
    }

    public double getLeftY() {
        return -super.getLeftY();
    }

    public double getRightY() {
        return -super.getRightY();
    }

}
