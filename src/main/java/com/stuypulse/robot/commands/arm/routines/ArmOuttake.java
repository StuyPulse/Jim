package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;

public class ArmOuttake extends ArmRoutine {
    
    public ArmOuttake() {
        super(Manager.getInstance()::getOuttakeTrajectory);
    }
}
