package com.stuypulse.robot.test;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.stuypulse.robot.constants.Motors.Swerve.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.robot.constants.Ports.Swerve.*;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.stuylib.network.SmartNumber;

public class SwerveDrive extends SubsystemBase {

    private SmartNumber turningVoltage;
    private SmartNumber drivingVoltage;

    private CANSparkMax turnMotorBL;
    private CANSparkMax turnMotorFR;
    private CANSparkMax turnMotorBR;
    private CANSparkMax turnMotorFL;
    private SparkMaxAbsoluteEncoder absoluteEncoderBL;
    private SparkMaxAbsoluteEncoder absoluteEncoderBR;
    private SparkMaxAbsoluteEncoder absoluteEncoderFL;
    private SparkMaxAbsoluteEncoder absoluteEncoderFR;

    private CANSparkMax driveMotorBL;
    private CANSparkMax driveMotorBR;
    private CANSparkMax driveMotorFL;
    private CANSparkMax driveMotorFR;
    private RelativeEncoder driveEncoderBL;
    private RelativeEncoder driveEncoderBR;
    private RelativeEncoder driveEncoderFL;
    private RelativeEncoder driveEncoderFR;

    public SwerveDrive() {

        turningVoltage = new SmartNumber("Swerve/Turning Voltage", 0.0);
        drivingVoltage = new SmartNumber("Swerve/Drive Voltage", 0.0);

        /** turn motors */
    
        turnMotorBL = new CANSparkMax(BackLeft.TURN, MotorType.kBrushless);
        turnMotorBR = new CANSparkMax(BackRight.TURN, MotorType.kBrushless);
        turnMotorFL = new CANSparkMax(FrontLeft.TURN, MotorType.kBrushless);
        turnMotorFR = new CANSparkMax(FrontRight.TURN, MotorType.kBrushless);

        /** drive motors */
        driveMotorBL = new CANSparkMax(BackLeft.DRIVE, MotorType.kBrushless);
        driveMotorBR = new CANSparkMax(BackRight.DRIVE, MotorType.kBrushless);
        driveMotorFL = new CANSparkMax(FrontLeft.DRIVE, MotorType.kBrushless);
        driveMotorFR = new CANSparkMax(FrontRight.DRIVE, MotorType.kBrushless);

        /** turn encoders */
        absoluteEncoderBL = turnMotorBL.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoderBR = turnMotorBR.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoderFL = turnMotorFL.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoderFR = turnMotorFR.getAbsoluteEncoder(Type.kDutyCycle);

        // drive encoders
        driveEncoderBL = driveMotorBL.getEncoder();
        driveEncoderBL.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoderBL.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

        driveEncoderBR = driveMotorBR.getEncoder();
        driveEncoderBR.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoderBR.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

        driveEncoderFL = driveMotorFL.getEncoder();
        driveEncoderFL.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoderFL.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);

        driveEncoderFR = driveMotorFR.getEncoder();
        driveEncoderFR.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoderFR.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        
        // configure motors
        TURN.configure(turnMotorBL);
        TURN.configure(turnMotorBR);
        TURN.configure(turnMotorFL);
        TURN.configure(turnMotorFR);

        DRIVE.configure(driveMotorBL);
        DRIVE.configure(driveMotorBR);
        DRIVE.configure(driveMotorFL);
        DRIVE.configure(driveMotorFR);
    }

    public void turnMotorBL() {
        turnMotorBL.setVoltage(turningVoltage.get());
    }

    public void turnMotorBR() {
        turnMotorBR.setVoltage(turningVoltage.get());
    }
    public void turnMotorFL() {
        turnMotorFL.setVoltage(turningVoltage.get());
    }
    
    public void turnMotorFR() {
        turnMotorFR.setVoltage(turningVoltage.get());
    }

    public void driveMotorBL() {
        driveMotorBL.setVoltage(drivingVoltage.get());
    }

    public void driveMotorBR() {
        driveMotorBR.setVoltage(drivingVoltage.get());
    }

    public void driveMotorFL() {
        driveMotorFL.setVoltage(drivingVoltage.get());
    }

    public void driveMotorFR() {
        driveMotorFR.setVoltage(drivingVoltage.get());
    }

    public void stop() {
        turnMotorBL.stopMotor();
        turnMotorBR.stopMotor();
        turnMotorFL.stopMotor();
        turnMotorFR.stopMotor();
        driveMotorBL.stopMotor();
        driveMotorBR.stopMotor();
        driveMotorFL.stopMotor();
        driveMotorFR.stopMotor();
    }

    @Override
    public void periodic() {
        // log encoder stuff
        SmartDashboard.putNumber("Swerve/Turn/Back Left", absoluteEncoderBL.getPosition());
        SmartDashboard.putNumber("Swerve/Turn/Back Right", absoluteEncoderBR.getPosition());
        SmartDashboard.putNumber("Swerve/Turn/Front Left", absoluteEncoderFL.getPosition());
        SmartDashboard.putNumber("Swerve/Turn/Front Right", absoluteEncoderFR.getPosition());

        SmartDashboard.putNumber("Swerve/Drive/Back Left Rotations", driveEncoderBL.getPosition());
        SmartDashboard.putNumber("Swerve/Drive/Back Right Rotations", driveEncoderBR.getPosition());
        SmartDashboard.putNumber("Swerve/Drive/Front Left Rotations", driveEncoderFL.getPosition());
        SmartDashboard.putNumber("Swerve/Drive/Front Right Rotations", driveEncoderFR.getPosition());

    }
}