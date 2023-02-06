package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager;

public class ArmReady extends ArmFollowTrajectory {

    private Manager manager;

    public ArmReady() {
        manager = Manager.getInstance();
        // add manager?
    }

    @Override
    public void initialize() {
        super.initialize(); // sus
        setTrajectory(manager.getReadyTrajectory());
    }
    
}
