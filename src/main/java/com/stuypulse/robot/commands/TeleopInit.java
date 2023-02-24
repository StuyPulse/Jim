package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.plant.Plant;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TeleopInit extends InstantCommand {

    private final Plant plant;

    public TeleopInit() {
        plant = Plant.getInstance();
    }

    @Override
    public void initialize() {
        plant.disengage();
    }
}
