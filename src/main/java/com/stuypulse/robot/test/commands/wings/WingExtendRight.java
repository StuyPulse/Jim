package com.stuypulse.robot.test.commands.wings;

import com.stuypulse.robot.test.subsystems.Wings;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingExtendRight extends InstantCommand{
    private Wings wings;

    public WingExtendRight(Wings wings){
        this.wings = wings;
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.extendRight();
    }

}
