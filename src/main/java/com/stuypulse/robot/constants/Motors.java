/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {

    int kDisableStatusFrame = 65535;

    public static void disableStatusFrames(CANSparkMax motor, int... ids) {
        for (int id : ids) {
            motor.setPeriodicFramePeriod(PeriodicFrame.fromId(id), kDisableStatusFrame);
        }
    }

    public interface Arm {
        CANSparkMaxConfig SHOULDER_LEFT_CONFIG = new CANSparkMaxConfig(true, IdleMode.kCoast, 40);
        CANSparkMaxConfig SHOULDER_RIGHT_CONFIG = new CANSparkMaxConfig(false, IdleMode.kCoast, 40);
        CANSparkMaxConfig WRIST_CONFIG = new CANSparkMaxConfig(true, IdleMode.kCoast, 40);
    }

    public interface Intake {
        CANSparkMaxConfig FRONT_MOTOR = new CANSparkMaxConfig(false, IdleMode.kBrake, 40, .1);
        CANSparkMaxConfig BACK_MOTOR = new CANSparkMaxConfig(true, IdleMode.kBrake, 40, .1);
    }

    public interface Swerve {
        CANSparkMaxConfig DRIVE = new CANSparkMaxConfig(false, IdleMode.kBrake, new CurrentLimit(60, 80), 0);
        CANSparkMaxConfig TURN  = new CANSparkMaxConfig(false, IdleMode.kBrake, new CurrentLimit(20, 40), 0);
    }

    /** Classes to store all of the values a motor needs */

    public static class TalonSRXConfig {
        public final boolean INVERTED;
        public final NeutralMode NEUTRAL_MODE;
        public final int PEAK_CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public TalonSRXConfig(
                boolean inverted,
                NeutralMode neutralMode,
                int peakCurrentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.NEUTRAL_MODE = neutralMode;
            this.PEAK_CURRENT_LIMIT_AMPS = peakCurrentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public TalonSRXConfig(boolean inverted, NeutralMode neutralMode, int peakCurrentLimitAmps) {
            this(inverted, neutralMode, peakCurrentLimitAmps, 0.0);
        }

        public TalonSRXConfig(boolean inverted, NeutralMode neutralMode) {
            this(inverted, neutralMode, 80);
        }

        public void configure(WPI_TalonSRX motor) {
            motor.setInverted(INVERTED);
            motor.setNeutralMode(NEUTRAL_MODE);
            motor.configContinuousCurrentLimit(PEAK_CURRENT_LIMIT_AMPS - 10, 0);
            motor.configPeakCurrentLimit(PEAK_CURRENT_LIMIT_AMPS, 0);
            motor.configPeakCurrentDuration(100, 0);
            motor.enableCurrentLimit(true);
            motor.configOpenloopRamp(OPEN_LOOP_RAMP_RATE);
        }
    }

    public static class VictorSPXConfig {
        public final boolean INVERTED;
        public final NeutralMode NEUTRAL_MODE;
        public final double OPEN_LOOP_RAMP_RATE;

        public VictorSPXConfig(
                boolean inverted,
                NeutralMode neutralMode,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.NEUTRAL_MODE = neutralMode;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public VictorSPXConfig(boolean inverted, NeutralMode neutralMode) {
            this(inverted, neutralMode, 0.0);
        }

        public void configure(WPI_VictorSPX motor) {
            motor.setInverted(INVERTED);
            motor.setNeutralMode(NEUTRAL_MODE);
            motor.configOpenloopRamp(OPEN_LOOP_RAMP_RATE);
        }
    }

    public static class CANSparkMaxConfig {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final CurrentLimit CURRENT_LIMIT;
        public final double OPEN_LOOP_RAMP_RATE;

        public CANSparkMaxConfig(
                boolean inverted,
                IdleMode idleMode,
                CurrentLimit currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode, int currentLimitAmps, double openLoopRampRate) {
            this(inverted, idleMode, new CurrentLimit(currentLimitAmps), openLoopRampRate);
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode, int currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.0);
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode, CurrentLimit currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.0);
        }

        public CANSparkMaxConfig(boolean inverted, IdleMode idleMode) {
            this(inverted, idleMode, new CurrentLimit());
        }

        public void configure(CANSparkMax motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT.CURRENT_LIMIT_AMPS, CURRENT_LIMIT.FREE_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.burnFlash();
         }
          
    }

    public static class CurrentLimit {
        public final int CURRENT_LIMIT_AMPS;
        public final int FREE_LIMIT_AMPS;

        public CurrentLimit(int currentLimitAmps, int freeLimitAmps) {
            CURRENT_LIMIT_AMPS = currentLimitAmps;
            FREE_LIMIT_AMPS = freeLimitAmps;
        }

        public CurrentLimit(int currentLimitAmps) {
            this(currentLimitAmps, 100);
        }

        public CurrentLimit() {
            this(80, 100);
        }
    }
}
