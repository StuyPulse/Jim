/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.wing;

import com.stuypulse.robot.subsystems.wing.Wing;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingRetract extends InstantCommand{

    public WingRetract() {
        super(() -> {
            if (Wing.getInstance().isExtended())
                Wing.getInstance().retract();
        });
    }

}
