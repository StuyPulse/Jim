package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TestInit extends InstantCommand {
    
    public TestInit() {
        addRequirements(Arm.getInstance());
    }

    @Override
    public void initialize() {
        var arm = Arm.getInstance();
        arm.enableShoulderBrakeMode();
        arm.enableWristBrakeMode();

        arm.setShoulderVoltage(0);
        arm.setWristVoltage(0);
    }

}
