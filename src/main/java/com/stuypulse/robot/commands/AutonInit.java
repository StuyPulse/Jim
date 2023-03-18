package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutonInit extends InstantCommand {
    
    public AutonInit() {
        addRequirements(Arm.getInstance());
    }

    @Override
    public void initialize() {
        var arm = Arm.getInstance();

        arm.enableShoulderBrakeMode();
        arm.enableWristBrakeMode();

        arm.setWristVoltage(0);
        arm.setShoulderVoltage(0);
    }

}
