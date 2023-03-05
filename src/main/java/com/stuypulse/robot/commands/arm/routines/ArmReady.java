package com.stuypulse.robot.commands.arm.routines;

import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.GamePiece;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.util.ArmState;
import com.stuypulse.robot.util.ArmTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

public class ArmReady extends ArmRoutine {
    
    public ArmReady() {
        super(Manager.getInstance()::getReadyTrajectory);
    }

    @Override
    protected ArmTrajectory getTrajectory(ArmState src, ArmState dest) {
        if (Manager.getInstance().getGamePiece() == GamePiece.CONE_TIP_UP) {
            return new ArmTrajectory()

                .addState(
                    new ArmState(dest.getShoulderDegrees(), src.getWristDegrees())
                        .setWristLimp(true))
                
                .addState(
                    dest.getShoulderDegrees(), 
                    (src.getWristDegrees() + dest.getWristDegrees()) / 2.0)
            
                .addState(dest);
        }

        return super.getTrajectory(src, dest);
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()) {
            return super.isFinished(); // Math.abs(Arm.getInstance().getWristVelocityRadiansPerSecond()) < Math.toRadians(5);
        } else {
            return super.isFinished();
        }
    }
}
