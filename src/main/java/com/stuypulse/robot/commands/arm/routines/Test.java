package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;

public class Test extends ArmFollowTrajectory {
    public Test() {
        // add manager?
        super(new Rotation2d(-30), new Rotation2d(60));
    }

    @Override
    public void initialize() {
        super.initialize(); // sus
    }
}
