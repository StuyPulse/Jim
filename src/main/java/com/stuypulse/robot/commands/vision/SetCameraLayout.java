/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.AbstractVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetCameraLayout extends InstantCommand {

    private final int[] tids;

    public SetCameraLayout(int[] tids) {
        this.tids = tids;
    }

    @Override
    public void initialize() {
        AbstractVision.getInstance().setCameraLayouts(tids);
    }
}
