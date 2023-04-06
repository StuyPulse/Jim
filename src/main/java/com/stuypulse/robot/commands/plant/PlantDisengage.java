/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.plant;

import com.stuypulse.robot.subsystems.plant.Plant;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PlantDisengage extends InstantCommand {

    private Plant plant;

    public PlantDisengage(){
        plant = Plant.getInstance();
        addRequirements(plant);
    }

    @Override
    public void initialize(){
        plant.disengage();
    }
}
