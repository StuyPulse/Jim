/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.plant;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Plant extends SubsystemBase {
    // Singleton
    private static final Plant instance;

    static {
        instance = Settings.ROBOT == Robot.JIM ? new PlantImpl() : new NoPlant();
    }

    public static Plant getInstance() {
        return instance;
    }

    // Plant methods
    protected Plant() {
    }

    public abstract void engage();
    public abstract void disengage();
    public abstract boolean isEngaged();
}
