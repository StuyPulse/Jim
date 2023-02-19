package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.Routine;
public class ArmReady extends ArmSetRoutine {
    
    public ArmReady() {   
        super(Routine.READY, Manager.getInstance()::getReadyTrajectory);
    }
}
