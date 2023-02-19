package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
public class ArmNeutral extends ArmRoutine {
    
    public ArmNeutral() {
        super(Routine.NEUTRAL, Manager.getInstance()::getNeutralTrajectory);
    }
}
