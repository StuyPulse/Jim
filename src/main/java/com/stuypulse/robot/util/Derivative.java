/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot.util;

import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.util.StopWatch;

/**
 * This class takes an IStream and gives you the derivative with respect to time.
 *
 * @author Sam (sam.belliveau@gmail.com)
 */
public class Derivative implements IFilter {

    private final StopWatch mTimer;
    private double mLastValue;
    private double mOutput;

    public Derivative() {
        mTimer = new StopWatch();
        mLastValue = 0.0;
    }

    public double get(double next) {
        double diff = next - mLastValue;
        mLastValue = next;
        mOutput = diff / mTimer.reset();
        return mOutput;
    }

    public double getOutput() {
        return mOutput;
    }

    public void reset() {
        mOutput = Double.POSITIVE_INFINITY;
    }
}
