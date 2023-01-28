package com.stuypulse.robot.commands.Wings;
import com.stuypulse.robot.subsystems.wings.IWings;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractRightWing extends CommandBase{
    private IWings wings;

    public RetractRightWing(){
        wings = IWings.getInstance();
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.retractRight();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
