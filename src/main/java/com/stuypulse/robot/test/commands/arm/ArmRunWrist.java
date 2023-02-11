package com.stuypulse.robot.test.commands.arm;


import com.stuypulse.robot.TestRobotContainer;
import com.stuypulse.robot.test.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmRunWrist extends InstantCommand{
    private Arm arm;

    public ArmRunWrist(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute(){
        arm.runWrist(TestRobotContainer.UNIVERSAL_VOLTAGE.get());
    }

}

