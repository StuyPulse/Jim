package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.util.ArmAngles;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmFollowPath extends SequentialCommandGroup {    
    public ArmFollowPath(ArmAngles... setpoints) {
        // for (ArmAngles setpoint : setpoints) {
        //     addCommands(new UpdateArm(Rotation2d.fromDegrees(setpoint.getShoulderDegrees()), 
        //                             Rotation2d.fromDegrees(setpoint.getWristDegrees())));
        // }
    }
}
