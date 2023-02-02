package com.stuypulse.robot.commands.plant;

import com.stuypulse.robot.subsystems.plant.IPlant;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PlantEngage extends InstantCommand{
    
    private IPlant plant;

    public PlantEngage(){
        plant = IPlant.getInstance();
        addRequirements(plant);
    }

    @Override
    public void initialize(){
        plant.engage();
    }
}
