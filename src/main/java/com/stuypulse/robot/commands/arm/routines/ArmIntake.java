package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;

public class ArmIntake extends ArmRoutine {
    
    public ArmIntake() {
        super(Manager.getInstance()::getIntakeTrajectory);
    }
}
