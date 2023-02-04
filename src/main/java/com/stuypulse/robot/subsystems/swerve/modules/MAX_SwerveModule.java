package com.stuypulse.robot.subsystems.swerve.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class MAX_SwerveModule extends ISwerveModule {

    // module data
    private final String id;
    private final Translation2d location;
    private SwerveModuleState targetState;

    // turn
    private final CANSparkMax turnMotor;
    private final SparkMaxAbsoluteEncoder absoluteEncoder;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    // controller
    private final SparkMaxPIDController drivePID;
    private final SimpleMotorFeedforward driveFF;
    private final SparkMaxPIDController turnPID;

    private double prevVelocity;
    
    public MAX_SwerveModule(String id, Translation2d location, int turnCANId, Rotation2d angleOffset, int driveCANId) {
        
        // module data
        this.id = id;
        this.location = location;

        // turn 
        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        
        absoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Encoder.Turn.POSITION_CONVERSION);
        absoluteEncoder.setVelocityConversionFactor(Encoder.Turn.VELOCITY_CONVERSION);
        absoluteEncoder.setZeroOffset(angleOffset.getRotations());
        absoluteEncoder.setInverted(true);

        turnPID = turnMotor.getPIDController();
        turnPID.setFeedbackDevice(absoluteEncoder);

        turnPID.setP(Turn.kP);
        turnPID.setI(Turn.kI);
        turnPID.setD(Turn.kD);
        turnPID.setOutputRange(-1, 1);

        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(Encoder.Turn.MIN_PID_INPUT);
        turnPID.setPositionPIDWrappingMaxInput(Encoder.Turn.MAX_PID_INPUT);

        // drive
        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        
        drivePID = driveMotor.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);

        drivePID.setP(Drive.kP);
        drivePID.setI(Drive.kI);
        drivePID.setD(Drive.kD);
        drivePID.setOutputRange(-1, 1);

        driveFF = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        
        targetState = new SwerveModuleState();

        prevVelocity = 0;

        driveMotor.enableVoltageCompensation(12.0);
        turnMotor.enableVoltageCompensation(12.0);
        Motors.Swerve.TURN.configure(turnMotor);
        Motors.Swerve.DRIVE.configure(turnMotor);
    }   
    
    @Override
    public String getID() {
        return id;
    }
    
    @Override
    public Translation2d getOffset() {
        return location;
    }
    
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }
    
    private double getVelocity() {
        return driveEncoder.getVelocity();
    }
    
    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getPosition());
    } 

    @Override 
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }
    
    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }



    @Override
    public void periodic() {
        // turn
        turnPID.setReference(targetState.angle.getRadians(), ControlType.kPosition);

        // drive
        double vel = getVelocity();
        double ffVoltage = driveFF.calculate(prevVelocity, vel, Settings.DT);
        drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kVelocity, 0, ffVoltage, ArbFFUnits.kVoltage);
        
        prevVelocity = vel;

        SmartDashboard.putNumber(id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber(id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(id + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(id + "/Velocity", vel);
    }
}