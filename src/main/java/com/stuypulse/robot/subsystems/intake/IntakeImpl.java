package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Motors.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Manager;
import com.stuypulse.robot.subsystems.Manager.IntakeSide;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BButton;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeImpl extends Intake{

    private CANSparkMax frontMotor; 
    private CANSparkMax backMotor;

    private DigitalInput frontSensor;
    private DigitalInput backSensor;

    private BStream stalling;
    private BStream newGamePiece;

    private BStream frontCube;
    private BStream backCube;

    private IntakeSide intookSide;

    private boolean deacquiring;

    public IntakeImpl(){
       
        frontSensor = new DigitalInput(FRONT_SENSOR);
        backSensor = new DigitalInput(BACK_SENSOR);

        frontMotor = new CANSparkMax(FRONT_MOTOR_PORT, MotorType.kBrushless);
        backMotor = new CANSparkMax(BACK_MOTOR_PORT, MotorType.kBrushless);

        FRONT_MOTOR.configure(frontMotor);
        BACK_MOTOR.configure(backMotor);

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounce.Rising(STALL_TIME));

        newGamePiece = BStream.create(this::hasGamePiece)
            .filtered(
                    new BButton.Pressed(),
                    new BDebounce.Falling(NEW_GAMEPIECE_TIME));

        frontCube = BStream.create(frontSensor::get).not()
            .filtered(new BDebounce.Rising(IR_SENSOR_TIME));

        backCube = BStream.create(backSensor::get).not()
            .filtered(new BDebounce.Rising(IR_SENSOR_TIME));

        intookSide = IntakeSide.FRONT;

        deacquiring = false;
    }

    // CONE DETECTION (stall detection)

    private double getMaxCurrent(){
        return Math.max(frontMotor.getOutputCurrent(), backMotor.getOutputCurrent());
    }

    private boolean isMomentarilyStalling() {
        return getMaxCurrent() > STALL_CURRENT.doubleValue();
    }

    private boolean isStalling() {
        return stalling.get();
    }

    // CUBE DETECTION (ir sensors)

    private boolean hasCubeFront() {
        return frontCube.get();
    }
    private boolean hasCubeBack() {
        return backCube.get();
    }
    private boolean hasCube() {
        return isFlipped() ? hasCubeBack() : hasCubeFront();
    }

    // GAMEPIECE DETECTION

    private boolean hasGamePiece() {
        return isStalling() || hasCube();
    }

    @Override
    public boolean hasNewGamePiece() {
        return newGamePiece.get();
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

    // NOTE: if called when wrist is on opposite side of final setpoint, direction will be wrong
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
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    @Override
    public IntakeSide getIntookSide() {
        return intookSide;
    }

    @Override
    public void periodic(){
        if (!deacquiring) {
            if (Manager.getInstance().getGamePiece().isCone() && isStalling()) {
                stop();
            }
    
            if (Manager.getInstance().getGamePiece().isCube() && hasCube()) {
                stop();
            }
        }

        if (hasNewGamePiece()) {
            intookSide = Manager.getInstance().getIntakeSide();
        }

        if (Settings.isDebug()) {
            Arm.getInstance().getVisualizer().setIntakingDirection(frontMotor.get(), backMotor.get());
       
            Settings.putNumber("Intake/Front Roller Speed", frontMotor.get());
            Settings.putNumber("Intake/Back Roller Speed", backMotor.get());
            Settings.putNumber("Intake/Front Roller Current", frontMotor.getOutputCurrent());
            Settings.putNumber("Intake/Back Roller Current", backMotor.getOutputCurrent());
            Settings.putBoolean("Intake/Is Flipped", isFlipped());
            Settings.putBoolean("Intake/Is Stalling", isStalling());
            Settings.putBoolean("Intake/Front IR Sensor", !frontSensor.get());
            Settings.putBoolean("Intake/Back IR Sensor", !backSensor.get());
            Settings.putBoolean("Intake/Has Cube Front", frontCube.get());
            Settings.putBoolean("Intake/Has Cube Back", backCube.get());
            Settings.putBoolean("Intake/Has Cube", hasCube());
            Settings.putBoolean("Intake/Deacquiring", deacquiring);

    
            Settings.putNumber("Intake/Front Motor", frontMotor.get());
            Settings.putNumber("Intake/Back Motor", backMotor.get());

            Settings.putString("Intake/Intook Side", intookSide.name());
        }
    }

}