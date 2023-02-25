package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.plant.Plant;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TeleopInit extends InstantCommand {

    private final Plant plant;
    private final Arm arm;

    public TeleopInit() {
        plant = Plant.getInstance();
        arm = Arm.getInstance();
    }

    @Override
    public void initialize() {
        plant.disengage();

        arm.setTargetState(arm.getState());
    }
}
