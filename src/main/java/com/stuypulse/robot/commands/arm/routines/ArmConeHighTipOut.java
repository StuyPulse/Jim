package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.commands.arm.ArmFollowTrajectory;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmConeHighTipOut extends ArmFollowTrajectory {

    public ArmConeHighTipOut(Arm arm) {
        super(() -> Manager.getInstance().getConeHighTipOutTrajectory(arm));
    }
    
}
