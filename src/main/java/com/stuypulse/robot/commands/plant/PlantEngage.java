/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.commands.plant;

import com.stuypulse.robot.subsystems.plant.Plant;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PlantEngage extends InstantCommand{

    private Plant plant;

    public PlantEngage(){
        plant = Plant.getInstance();
        addRequirements(plant, SwerveDrive.getInstance());
    }

    @Override
    public void initialize(){
        SwerveDrive.getInstance().setXMode();
        plant.engage();
    }
}
