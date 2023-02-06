package com.stuypulse.robot.commands.plant;

import com.stuypulse.robot.subsystems.plant.Plant;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PlantEngage extends InstantCommand{
    
    private Plant plant;

    public PlantEngage(){
        plant = Plant.getInstance();
        addRequirements(plant);
    }

    @Override
    public void initialize(){
        plant.engage();
    }
}
