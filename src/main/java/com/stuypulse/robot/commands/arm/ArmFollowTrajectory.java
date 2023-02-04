package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.util.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmFollowTrajectory extends SequentialCommandGroup {    
    public ArmFollowTrajectory(ArmTrajectory trajectory) {
        for (ArmState setpoint : trajectory.getStates()) {
            addCommands(new UpdateArm(setpoint.getShoulderRotation(), 
                                    setpoint.getWristRotation()));
        }
    }
}
