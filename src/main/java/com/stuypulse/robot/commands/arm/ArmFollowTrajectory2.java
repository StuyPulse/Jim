package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.util.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmFollowTrajectory2 extends SequentialCommandGroup {    
    public ArmFollowTrajectory2(ArmTrajectory trajectory) {
        for (ArmState setpoint : trajectory.getStates()) {
            addCommands(new ArmReachSetpoint(setpoint.getShoulderState(), setpoint.getWristState()));
        }
    }
}
