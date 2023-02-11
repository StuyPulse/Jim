package com.stuypulse.robot.test.commands.plant;

import com.stuypulse.robot.test.subsystems.Plant;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PlantEngage extends InstantCommand {
    
    private Plant plant;

    public PlantEngage(Plant plant){
        this.plant = plant;
        addRequirements(plant);
    }

    @Override
    public void initialize() {
        plant.engage();
    }

}
