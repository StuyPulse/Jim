package com.stuypulse.robot.commands.plant;

import com.stuypulse.robot.subsystems.plant.Plant;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class engage extends CommandBase{
    
    public final Plant plant;

    public engage(Plant plant){
        this.plant = plant;
        addRequirements(plant);
    }

    @Override
    public void initialize(){
        plant.engage();
    }
}
