package com.stuypulse.robot.subsystems.wings;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Wings extends SubsystemBase {

    private static final Wings instance;

    static {
        instance = Settings.ROBOT == Robot.JIM ? new WingsImpl() : new NoWings();
    }

    public static Wings getInstance() {
        return instance;
    }


    public abstract boolean isRedExtended();
    public abstract boolean isWhiteExtended();

    public abstract void extendRed();
    public abstract void retractRed();
    public abstract void extendWhite();
    public abstract void retractWhite();
}
