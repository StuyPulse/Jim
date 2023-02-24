package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;

public class ArmNeutral extends ArmRoutine {
    
    public ArmNeutral() {
        super(Manager.getInstance()::getNeutralTrajectory);
    }
}
