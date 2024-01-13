/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.numbers.filters.Derivative;

public class Feedthrough extends Controller {

    private Derivative derivative;

    public Feedthrough() {
        derivative = new Derivative();
    }

    @Override
    protected double calculate(double setpoint, double measurement) {
        return derivative.get(setpoint);
    }

}
