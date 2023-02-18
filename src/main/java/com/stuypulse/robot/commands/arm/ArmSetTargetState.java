package com.stuypulse.robot.commands.arm;

import java.util.function.Supplier;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmSetTargetState extends InstantCommand {
    public ArmSetTargetState(Supplier<ArmState> targetSupplier) {
        super(() -> {
            Arm.getInstance().setTargetState(targetSupplier.get());
        });
    }
}
