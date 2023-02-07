package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager;

public class ArmScore extends ArmFollowTrajectory {

    private Manager manager;

    public ArmScore() {
        manager = Manager.getInstance();
        // add manager?
    }

    @Override
    public void initialize() {
        super.initialize(); // sus
        setTrajectory(manager.getScoreTrajectory());
    }
    
}