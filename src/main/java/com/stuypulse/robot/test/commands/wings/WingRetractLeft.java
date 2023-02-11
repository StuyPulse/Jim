package com.stuypulse.robot.test.commands.wings;

import com.stuypulse.robot.test.subsystems.Wings;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingRetractLeft extends InstantCommand{
    private Wings wings;

    public WingRetractLeft(Wings wings){
        this.wings = wings;
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.retractLeft();
    }

}
