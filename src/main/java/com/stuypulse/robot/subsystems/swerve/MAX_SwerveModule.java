package com.stuypulse.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import com.stuypulse.robot.subsystems.ISwerveModule;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.network.SmartAngle;

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
    private final SmartAngle angleOffset;

    // drive
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;

    // controller
    private final SparkMaxPIDController drivePID;
    private final Controller driveFF;
    private final SparkMaxPIDController turnController;
    
    public MAX_SwerveModule(String id, Translation2d location, int turnCANId, SmartAngle angleOffset, int driveCANId) {
        
        // module data
        this.id = id;
        this.location = location;

        // turn 
        turnMotor = new CANSparkMax(turnCANId, MotorType.kBrushless);
        Motors.Swerve.TURN.configure(turnMotor); // constant
        
        // double check this
        absoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turnController = turnMotor.getPIDController();
        turnController.setFeedbackDevice(absoluteEncoder);

        turnController.setP(Turn.kP.doubleValue());
        turnController.setI(Turn.kI.doubleValue());
        turnController.setD(Turn.kD.doubleValue());

        turnController.setPositionPIDWrappingEnabled(true);
        turnController.setPositionPIDWrappingMinInput(-Math.PI);
        turnController.setPositionPIDWrappingMaxInput(+Math.PI);

        // drive
        driveMotor = new CANSparkMax(driveCANId, MotorType.kBrushless);
        Motors.Swerve.DRIVE.configure(turnMotor); 
        
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Encoder.Drive.POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Encoder.Drive.VELOCITY_CONVERSION);
        
        drivePID = driveMotor.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);

        drivePID.setP(Drive.kP.doubleValue());
        drivePID.setI(Drive.kI.doubleValue());
        drivePID.setD(Drive.kD.doubleValue());

        driveFF = new Feedforward.Motor(Drive.kS, Drive.kV, Drive.kA).velocity();
        
        targetState = new SwerveModuleState();
        this.angleOffset = angleOffset;
    }   
    
    @Override
    public String getID(){
        return id;
    }
    
    @Override
    public Translation2d getLocation() {
        return location;
    }
    
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getAngle());
    }
    
    private double getSpeed() {
        return driveEncoder.getVelocity();
    }
    
    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(absoluteEncoder.getPosition()).minus(angleOffset.getRotation2d());
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
        turnController.setReference(targetState.angle.getRadians(), ControlType.kPosition);

        // drive
        double ffVoltage = driveFF.update(targetState.speedMetersPerSecond, getSpeed());
        drivePID.setReference(targetState.speedMetersPerSecond, ControlType.kPosition, 0, ffVoltage, ArbFFUnits.kVoltage);
        
        SmartDashboard.putNumber(id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber(id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(id + "/Absolute Angle", absoluteEncoder.getPosition() * 360);
        SmartDashboard.putNumber(id + "/Target Speed", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(id + "/Speed", getSpeed());
    }
}