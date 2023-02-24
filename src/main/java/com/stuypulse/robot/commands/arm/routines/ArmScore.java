package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;

public class ArmScore extends ArmRoutine {
    
    public ArmScore() {
        super(Manager.getInstance()::getScoreTrajectory);
    }
}
