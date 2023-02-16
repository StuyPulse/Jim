package com.stuypulse.robot.subsystems.wings;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Wings extends SubsystemBase {

    private static Wings instance;

    public static Wings getInstance() {
        if (instance == null) {
            instance = Settings.ROBOT == Robot.JIM ? new WingsImpl() : new NoWings();
        }
        return instance;
    }


    public abstract boolean isLeftExtended();
    public abstract boolean isRightExtended();

    public abstract void extendLeft();
    public abstract void retractLeft();
    public abstract void extendRight();
    public abstract void retractRight();
}
