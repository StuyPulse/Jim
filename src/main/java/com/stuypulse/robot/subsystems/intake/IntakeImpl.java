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
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BButton;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeImpl extends Intake{

    private CANSparkMax frontMotor; 
    private CANSparkMax backMotor;

    private DigitalInput frontSensor;
    private DigitalInput backSensor;

    private BStream stalling;
    private BStream newGamePiece;

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

        newGamePiece =
                BStream.create(this::hasGamePiece)
                        .filtered(
                                new BButton.Pressed(),
                                new BDebounce.Falling(Settings.Intake.NEW_GAMEPIECE_TIME));
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
        return !frontSensor.get();
    }
    private boolean hasCubeBack() {
        return !backSensor.get();
    }
    private boolean hasCube() {
        return isFlipped() ? hasCubeBack() : hasCubeFront();
    }

    // GAMEPIECE DETECTION

    private boolean hasGamePiece() {
        return isStalling() || hasCube();
    }

    // WRIST ORIENTATION

    private boolean isFlipped() {
        return Arm.getInstance().getWristAngle().getDegrees() > 90 || Arm.getInstance().getWristAngle().getDegrees() < -90;
    }

    // INTAKING MODES

    @Override
    public void stop(){
        frontMotor.stopMotor();
        backMotor.stopMotor();
    }

    @Override
    public void acquireCube(){
        deacquiring = false;
        
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
        deacquiring = false;
        
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
        deacquiring = true;
        
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
        deacquiring = true;
        
        if (Manager.getInstance().getIntakeSide() == IntakeSide.FRONT) {
            frontMotor.set(OUTTAKE_CONE_FRONT_ROLLER.get());
            backMotor.set(-OUTTAKE_CONE_BACK_ROLLER.get());
        } else {
            frontMotor.set(-OUTTAKE_CONE_FRONT_ROLLER.get());
            backMotor.set(OUTTAKE_CONE_BACK_ROLLER.get());
        }
    }

    @Override
    public boolean hasNewGamePiece() {
        return newGamePiece.get();
    }
    
    @Override
    public void periodic(){
        stalling.get();
        newGamePiece.get();

        // if (!deacquiring && (isStalling() || hasCube())) {
        //     stop();
        // }
        if (isStalling()) {
            stop();
        }

        if (Settings.isDebug()) {
            Arm.getInstance().getVisualizer().setIntakingDirection(frontMotor.get(), backMotor.get());
       
            Settings.putNumber("Intake/Front Roller Speed", frontMotor.get());
            Settings.putNumber("Intake/Back Roller Speed", backMotor.get());
            Settings.putNumber("Intake/Front Roller Current", frontMotor.getOutputCurrent());
            Settings.putNumber("Intake/Back Roller Current", backMotor.getOutputCurrent());
            Settings.putBoolean("Intake/Is Flipped", isFlipped());
            Settings.putBoolean("Intake/Is Stalling", isStalling());
            Settings.putBoolean("Intake/Has Cube", hasCube());
    
            Settings.putNumber("Intake/Front Motor", frontMotor.get());
            Settings.putNumber("Intake/Back Motor", backMotor.get());
        }
    }

}