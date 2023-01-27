package com.stuypulse.robot.subsystems.intake;

import static com.stuypulse.robot.constants.Motors.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Ports.Intake.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.IArm;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends IIntake{

    private CANSparkMax frontMotor; 
    private CANSparkMax backMotor;

    private DigitalInput frontLeftSensor;
    private DigitalInput frontRightSensor;
    private DigitalInput backLeftSensor;
    private DigitalInput backRightSensor;

    private BStream stalling;

    public GamePiece gamePiece;

    public Intake(){
       
        frontMotor = new CANSparkMax(FRONT_MOTOR_PORT, MotorType.kBrushless);
        backMotor = new CANSparkMax(BACK_MOTOR_PORT, MotorType.kBrushless);

        FRONT_MOTOR.configure(frontMotor);
        BACK_MOTOR.configure(backMotor);

        stalling = BStream.create(this::isMomentarilyStalling)
            .filtered(new BDebounce.Rising(STALL_TIME))
            .polling(Settings.DT);

        frontLeftSensor = new DigitalInput(FRONT_LEFT_SENSOR);
        frontRightSensor = new DigitalInput(FRONT_RIGHT_SENSOR);
        backLeftSensor = new DigitalInput(BACK_LEFT_SENSOR);
        backRightSensor = new DigitalInput(BACK_RIGHT_SENSOR);
    }

    // CONE DETECTION (stall detection)

    private double maxCurrentDraw(){
        return frontMotor.getOutputCurrent() > backMotor.getOutputCurrent()? frontMotor.getOutputCurrent() : backMotor.getOutputCurrent();
    }

    private boolean isMomentarilyStalling() {
        return maxCurrentDraw() > STALL_CURRENT.doubleValue();
    }

    private boolean isStalling() {
        return stalling.get();
    }

    // CUBE DETECTION (ir sensors)

    private boolean hasCubeFront() {
        return !frontLeftSensor.get()||!frontRightSensor.get();
    }
    private boolean hasCubeBack() {
        return !backLeftSensor.get()||!backRightSensor.get();
    }
    private boolean hasCube() {
        return isFlipped()? hasCubeBack() : hasCubeFront();
    }

    // WRIST ORIENTATION

    private boolean isFlipped() {
        IArm arm = IArm.getInstance();
        return arm.getWristAngle().getDegrees() > 90 || arm.getWristAngle().getDegrees() < -90;
    }

    // INTAKING MODES

    @Override
    public void cubeIntake(){
        if (isFlipped()) {
            frontMotor.set(-CUBE_FRONT_ROLLER.get());
            backMotor.set(-CUBE_BACK_ROLLER.get());
        } else {
            frontMotor.set(CUBE_FRONT_ROLLER.get());
            backMotor.set(CUBE_BACK_ROLLER.get());
        }
    }

    @Override
    public void coneIntake() {
        if (isFlipped()) {
            frontMotor.set(-CONE_FRONT_ROLLER.get());
            backMotor.set(CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(CONE_FRONT_ROLLER.get());
            backMotor.set(-CONE_BACK_ROLLER.get());
        }
    }

    @Override
    public void cubeOuttake(){
        if (isFlipped()) {
            frontMotor.set(CUBE_FRONT_ROLLER.get());
            backMotor.set(CUBE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-CUBE_FRONT_ROLLER.get());
            backMotor.set(-CUBE_BACK_ROLLER.get());
        }
    }

    @Override
    public void coneOuttake(){
        if (isFlipped()) {
            frontMotor.set(CONE_FRONT_ROLLER.get());
            backMotor.set(-CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-CONE_FRONT_ROLLER.get());
            backMotor.set(CONE_BACK_ROLLER.get());
        }
    }

    @Override
    public void periodic(){
        if (isStalling() || hasCube()) {
            frontMotor.stopMotor();
            backMotor.stopMotor();
        }

        if(hasCube()){
            gamePiece = GamePiece.cube;
        } else if(isStalling()){
            gamePiece = GamePiece.cone;
        }
        
        SmartDashboard.putNumber("Intake/Front Roller Current", frontMotor.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Back Roller Current", backMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Intake/Is Flipped", isFlipped());
        SmartDashboard.putBoolean("Intake/Is Stalling", isStalling());
        SmartDashboard.putBoolean("Intake/Has Cube", hasCube());
    }

}