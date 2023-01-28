package com.stuypulse.robot.commands.Wings;
import com.stuypulse.robot.subsystems.wings.IWings;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendRightWing extends CommandBase{
    private IWings wings;

    public ExtendRightWing(){
        wings = IWings.getInstance();
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.extendRight();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
