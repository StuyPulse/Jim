/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.util.StopWatch;

public class AngleVelocity {

    private final StopWatch mTimer;
    private Angle mPreviousAngle;
    private double velocity;

    public AngleVelocity() {
        mTimer = new StopWatch();
        mPreviousAngle = Angle.kZero;
        velocity = 0;
    }

    public double update(Angle angle) {
        double velocity = angle.velocityRadians(mPreviousAngle, mTimer.reset());
        mPreviousAngle = angle;
        return this.velocity = velocity;
    }

    public double getVelocityRadiansPerSecond() {
        return velocity;
    }
}
