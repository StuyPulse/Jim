package com.stuypulse.robot.test.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.robot.constants.Ports.Swerve.*;

public class SwerveDrive extends SubsystemBase {

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
        driveEncoderBR = driveMotorBR.getEncoder();
        driveEncoderFL = driveMotorFL.getEncoder();
        driveEncoderFR = driveMotorFR.getEncoder();


    }

    public void turnMotorBL(double voltage) {
        turnMotorBL.setVoltage(voltage);
    }

    public void turnMotorBR(double voltage) {
        turnMotorBR.setVoltage(voltage);
    }
    public void turnMotorFL(double voltage) {
        turnMotorFL.setVoltage(voltage);
    }
    
    public void turnMotorFR(double voltage) {
        turnMotorFR.setVoltage(voltage);
    }

    public void driveMotorBL(double voltage) {
        driveMotorBL.setVoltage(voltage);
    }

    public void driveMotorBR(double voltage) {
        driveMotorBR.setVoltage(voltage);
    }

    public void driveMotorFL(double voltage) {
        driveMotorFL.setVoltage(voltage);
    }

    public void driveMotorFR(double voltage) {
        driveMotorFR.setVoltage(voltage);
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