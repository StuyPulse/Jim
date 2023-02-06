package com.stuypulse.robot.commands.wings;

import com.stuypulse.robot.subsystems.wings.IWings;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ExtendLeftWing extends InstantCommand{
    private IWings wings;

    public ExtendLeftWing(){
        wings = IWings.getInstance();
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.extendLeft();
    }

}
