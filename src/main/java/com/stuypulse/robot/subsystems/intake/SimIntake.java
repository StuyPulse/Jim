package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Settings.Intake.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

public class SimIntake extends Intake {

    SmartNumber frontMotor = new SmartNumber("Intake/Front Motor", 0);
    SmartNumber backMotor = new SmartNumber("Intake/Back Motor", 0);

    SmartBoolean hasNewGamePiece = new SmartBoolean("Intake/Has New Gamepiece", false);

    private boolean isFlipped() {
        Arm arm = Arm.getInstance();
        return arm.getWristAngle().getDegrees() > 90 || arm.getWristAngle().getDegrees() < -90;
    }

    @Override
    public void acquireCube(){
        if (isFlipped()) {
            frontMotor.set(-INTAKE_CUBE_FRONT_ROLLER.get());
            backMotor.set(-INTAKE_CUBE_BACK_ROLLER.get());
        } else {
            frontMotor.set(INTAKE_CUBE_FRONT_ROLLER.get());
            backMotor.set(INTAKE_CUBE_BACK_ROLLER.get());
        }
    }

    @Override
    public void acquireCone() {
        if (isFlipped()) {
            frontMotor.set(-INTAKE_CONE_FRONT_ROLLER.get());
            backMotor.set(INTAKE_CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(INTAKE_CONE_FRONT_ROLLER.get());
            backMotor.set(-INTAKE_CONE_BACK_ROLLER.get());
        }
    }

    @Override
    public void deacquireCube(){
        if (isFlipped()) {
            frontMotor.set(OUTTAKE_CUBE_FRONT_ROLLER.get());
            backMotor.set(OUTTAKE_CUBE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-OUTTAKE_CUBE_FRONT_ROLLER.get());
            backMotor.set(-OUTTAKE_CUBE_BACK_ROLLER.get());
        }
    }

    @Override
    public void deacquireCone(){
        if (Manager.getInstance().getIntakeSide() == IntakeSide.BACK) {
            frontMotor.set(OUTTAKE_CONE_FRONT_ROLLER.get());
            backMotor.set(-OUTTAKE_CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-OUTTAKE_CONE_FRONT_ROLLER.get());
            backMotor.set(OUTTAKE_CONE_BACK_ROLLER.get());
        }
    }

    @Override
    public void stop() {
        frontMotor.set(0);
        backMotor.set(0);
    }

    @Override
    public boolean hasNewGamePiece() {
        return hasNewGamePiece.get();
    }

    @Override
    public void periodic() {
        if (Settings.isDebug()) {
            Arm.getInstance().getVisualizer().setIntakingDirection(frontMotor.get(), backMotor.get());
        }
    }
    
}
