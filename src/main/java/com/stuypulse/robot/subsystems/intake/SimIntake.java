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

    private IntakeSide intookSide;

    private boolean deacquiring;

    public SimIntake() {
        intookSide = IntakeSide.FRONT;
        deacquiring = false;
    }

    // GAMEPIECE DETECTION

    @Override
    public boolean hasNewGamePiece() {
        return hasNewGamePiece.get();
    }

    // WRIST ORIENTATION

    private boolean isFlipped() {
        Arm arm = Arm.getInstance();
        return arm.getWristAngle().getDegrees() > 90 || arm.getWristAngle().getDegrees() < -90;
    }

    // INTAKING MODES

    private void setState(double frontSpeed, double backSpeed, boolean isCone, boolean flipped) {
        if (flipped) {
            frontSpeed *= -1;
            backSpeed *= -1;
        }

        if (isCone) {
            backSpeed *= -1;
        }

        frontMotor.set(frontSpeed);
        backMotor.set(backSpeed);
    }

    @Override
    public void acquireCube() {
        deacquiring = false;
        setState(+INTAKE_CUBE_ROLLER_FRONT.get(), +INTAKE_CUBE_ROLLER_BACK.get(), false, Manager.getInstance().getIntakeSide() == IntakeSide.BACK);
    }

    @Override
    public void acquireCone() {
        deacquiring = false;
        setState(+INTAKE_CONE_ROLLER_FRONT.get(), +INTAKE_CONE_ROLLER_BACK.get(), true, Manager.getInstance().getIntakeSide() == IntakeSide.BACK);
    }

    @Override
    public void deacquireCube() {
        deacquiring = true;
        setState(-OUTTAKE_CUBE_ROLLER_FRONT.get(), -OUTTAKE_CUBE_ROLLER_BACK.get(), false, isFlipped());
    }

    @Override
    public void deacquireCone() {
        deacquiring = true;

        setState(-OUTTAKE_CONE_ROLLER_FRONT.get(), -OUTTAKE_CONE_ROLLER_BACK.get(), true, intookSide == IntakeSide.BACK);
    }

    @Override
    public void stop() {
        frontMotor.set(0);
        backMotor.set(0);
    }

    @Override
    public void periodic() {
        if (hasNewGamePiece()) {
            intookSide = Manager.getInstance().getIntakeSide();
        }

        if (Settings.isDebug()) {
            Arm.getInstance().getVisualizer().setIntakingDirection(frontMotor.get(), backMotor.get());
        }
    }
    
}
