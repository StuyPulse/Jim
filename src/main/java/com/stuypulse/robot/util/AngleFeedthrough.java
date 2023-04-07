/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.util.AngleVelocity;

public class AngleFeedthrough extends AngleController {

    private AngleVelocity derivative;

    public AngleFeedthrough() {
        derivative = new AngleVelocity();
    }

    @Override
    protected double calculate(Angle setpoint, Angle measurement) {
        return derivative.get(setpoint);
    }

}
