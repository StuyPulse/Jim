package com.stuypulse.robot.commands.Wings;
import com.stuypulse.robot.subsystems.wings.IWings;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractLeftWing extends CommandBase{
    private IWings wings;

    public RetractLeftWing(){
        wings = IWings.getInstance();
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.retractLeft();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
