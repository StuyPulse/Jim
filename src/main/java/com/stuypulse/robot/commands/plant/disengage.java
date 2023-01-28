package com.stuypulse.robot.commands.plant;

import com.stuypulse.robot.subsystems.plant.Plant;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class disengage extends CommandBase{
    
    public final Plant plant;

    public disengage(Plant plant){
        this.plant = plant;
        addRequirements(plant);
    }

    @Override
    public void initialize(){
        plant.disengage();
    }
}
