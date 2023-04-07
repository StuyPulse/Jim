/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.streams.IStream;

public class ArmDriveFeedforward extends Controller {

    private final Number kG;
    private final IStream forwardAccelerationInGs;

    public ArmDriveFeedforward(Number kG, IStream forwardAccelerationInGs) {
        this.kG = kG;
        this.forwardAccelerationInGs = forwardAccelerationInGs;
    }

    @Override
    protected double calculate(double setpoint, double measurement) {
        return kG.doubleValue() * Math.sin(measurement) * forwardAccelerationInGs.get();
    }

}
