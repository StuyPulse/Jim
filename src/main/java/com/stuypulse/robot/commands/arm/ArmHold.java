package com.stuypulse.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.stuypulse.robot.subsystems.arm.*;

public class ArmHold extends InstantCommand {

    public ArmHold() {
        super(() -> {
            Arm arm = Arm.getInstance();
            arm.setTargetState(arm.getState());
        });
    }
}