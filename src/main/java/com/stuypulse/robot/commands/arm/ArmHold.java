package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmHold extends InstantCommand {
    
    public ArmHold() {
        super(() -> {
            var arm = Arm.getInstance();

            arm.setTargetState(arm.getState());
        });
    }

}
