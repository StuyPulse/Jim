package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
public class ArmReady extends ArmRoutine {
    
    public ArmReady() {
        super(Manager.getInstance()::getReadyTrajectory);
    }
}
