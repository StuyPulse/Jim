package com.stuypulse.robot.subsystems.wing;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Wing extends SubsystemBase {

    private static final Wing instance;

    static {
        instance = Settings.ROBOT == Robot.JIM ? new WingImpl() : new NoWing();
    }

    public static Wing getInstance() {
        return instance;
    }

    public abstract boolean isExtended();

    public abstract void extend();
    public abstract void retract();
}
