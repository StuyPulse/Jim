/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot.util; 

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.angles.filters.AFilter;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A filter, that when applied to the input of a motor, will profile it. Similar to the way in which
 * motion profiling can limit the amount of acceleration and jerk in an S-Curve, this can do that to
 * real time input. Because this will add a delay, it is recommended that the accelLimit is as high
 * as possible. Aside from the accelLimit, this is identical to a SlewRateLimiter or TimedRateLimit.
 *
 * @author Sam (sam.belliveau@gmail.com)
 */
public class AngleMotionProfile {

    // Default number of times to apply filter (helps accuracy)
    private static final int kDefaultSteps = 64;

    // Stopwatch to Track dt
    private StopWatch mTimer;

    // Limits for each of the derivatives
    private Number mVelLimit;
    private Number mAccelLimit;

    // The last output / acceleration
    private Angle mPosition;
    private double mVelocity;


    private double mAccel;
    // Number of times to apply filter (helps accuracy)
    private final int mSteps;

    /**
     * @param velLimit maximum change in velocity per second (u/s)
     * @param accelLimit maximum change in acceleration per second (u/s/s)
     * @param steps number of times to apply filter (improves accuracy)
     */
    public AngleMotionProfile(Number velLimit, Number accelLimit) {
        mTimer = new StopWatch();

        mVelLimit = velLimit;
        mAccelLimit = accelLimit;

        mPosition = Angle.kZero;
        mVelocity = 0;
        mAccel = 0;

        mSteps = kDefaultSteps;
    }

    public void setPosition(Angle position) {
        mPosition = position;
    }

    public void setVelocity(double velocity) {
        mVelocity = velocity;
    }

    public static class State {
        public Angle position;
        public double velocity;
        public double acceleration;

        public State(Angle position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    public State get(Rotation2d rotation2d) { 
        return get(Angle.fromRotation2d(rotation2d));
    }

    public State get(Angle target) {
        double period = mTimer.reset();
        double dt = period / mSteps;

        double velocityBefore = mVelocity;

        for (int i = 0; i < mSteps; ++i) {
            // if there is a accel limit, limit the amount the acceleration can change
            if (0 < mAccelLimit.doubleValue()) {
                // amount of windup in system (how long it would take to slow down)
                double windup = Math.abs(mVelocity) / mAccelLimit.doubleValue();

                // If the windup is too small, just use normal algorithm to limit acceleration
                if (windup < dt) {
                    // Calculate acceleration needed to reach target
                    double accel = target.velocityRadians(mPosition, dt) - mVelocity;

                    // Try to reach it while abiding by accellimit
                    mVelocity += SLMath.clamp(accel, dt * mAccelLimit.doubleValue());
                } else {
                    // the position it would end up if it attempted to come to a full stop
                    Angle future =
                            mPosition.addRadians(
                                    0.5 * mVelocity * (dt + windup)); // where the robot will end up

                    // Calculate acceleration needed to come to stop at target throughout windup
                    double accel = target.velocityRadians(future, windup);

                    // Try to reach it while abiding by accelLimit
                    mVelocity += SLMath.clamp(accel, dt * mAccelLimit.doubleValue());
                }

            } else {
                // make the acceleration the difference between target and current
                mVelocity = target.velocityRadians(mPosition, dt);
            }

            // if there is an acceleration limit, limit the acceleration
            if (0 < mVelLimit.doubleValue()) {
                mVelocity = SLMath.clamp(mVelocity, mVelLimit.doubleValue());
            }

            // adjust output by calculated acceleration
            mPosition = mPosition.addRadians(dt * mVelocity);
        }

        return new State(mPosition, velocityBefore, (mVelocity - velocityBefore) / period);
    }
}
