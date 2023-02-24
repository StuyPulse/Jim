package com.stuypulse.robot.commands.arm.routines;

import static com.stuypulse.robot.constants.ArmTrajectories.*;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmNeutral extends ArmRoutine {
    
    public static ArmTrajectory generateTrajectory(ArmState src, ArmState dest) {
        ArmTrajectory out = new ArmTrajectory();

        ShoulderState srcState = ShoulderState.getFrontBackState(src.getShoulderState());
        ShoulderState destState = ShoulderState.getState(dest.getShoulderState());

        SmartDashboard.putString("Arm/Shoulder/Source State", srcState.name());
        SmartDashboard.putString("Arm/Shoulder/Destination State", destState.name());

        // no safe points necessary
        if (srcState == destState) {
            return out.addState(dest);
        }

        // if trying to cross, add both safety poitns
        if (srcState.isOppositeFrom(destState)) {
            if (srcState == ShoulderState.FRONT) {
                out.addState(kSafePointFront);
                out.addState(kSafePointBack);
            } else {
                out.addState(kSafePointBack);
                out.addState(kSafePointFront);
            }
        }

        // if starting within bumper or ending within bumper, add one safety point
        else {
            if (srcState == ShoulderState.FRONT || destState == ShoulderState.FRONT) {
                out.addState(kSafePointFront);
            } else {
                out.addState(kSafePointBack);
            }
        }

        return out.addState(dest);
    }
    
    public ArmNeutral() {
        super(Manager.getInstance()::getNeutralTrajectory);
    }

    @Override
    public void initialize() {
        super.initialize();

        var state = Arm.getInstance().getState();

        trajectory = generateTrajectory(
            state,
            Manager.getInstance().getNeutralTrajectory()).wristMovesFirst(state);
    }
}
