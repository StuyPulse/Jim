package com.stuypulse.robot.commands.wings;

import com.stuypulse.robot.subsystems.wings.Wings;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingExtendLeft extends InstantCommand{
    private Wings wings;

    public WingExtendLeft(){
        wings = Wings.getInstance();
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.extendLeft();
    }

}
