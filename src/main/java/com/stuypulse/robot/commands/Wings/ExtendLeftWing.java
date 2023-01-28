package com.stuypulse.robot.commands.Wings;
import com.stuypulse.robot.subsystems.wings.IWings;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendLeftWing extends CommandBase{
    private IWings wings;

    public ExtendLeftWing(){
        wings = IWings.getInstance();
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.extendLeft();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
