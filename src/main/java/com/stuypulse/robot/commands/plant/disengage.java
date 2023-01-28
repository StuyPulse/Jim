package com.stuypulse.robot.commands.plant;

import com.stuypulse.robot.subsystems.plant.Plant;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Disengage extends InstantCommand{
    
    public final Plant plant;

    public Disengage(Plant plant){
        this.plant = plant;
        addRequirements(plant);
    }

    @Override
    public void initialize(){
        plant.disengage();
    }
}
