/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.plant;

public class NoPlant extends Plant {

    private boolean isEngaged;

    protected NoPlant() {
        isEngaged = false;
    }

    @Override
    public void engage() {
        isEngaged = true;
    }

    @Override
    public void disengage() {
        isEngaged = false;
    }

    @Override
    public boolean isEngaged() {
        return isEngaged;
    }

}
