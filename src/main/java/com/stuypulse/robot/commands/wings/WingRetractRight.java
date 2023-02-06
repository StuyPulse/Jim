package com.stuypulse.robot.commands.wings;
import com.stuypulse.robot.subsystems.wings.Wings;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class WingRetractRight extends InstantCommand{
    private Wings wings;

    public WingRetractRight(){
        wings = Wings.getInstance();
        addRequirements(wings);
    }

    @Override
    public void initialize(){
        wings.retractRight();
    }
}
