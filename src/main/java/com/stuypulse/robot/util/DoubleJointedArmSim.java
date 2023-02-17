package com.stuypulse.robot.util;

public class DoubleJointedArmSim {

    private final SingleJointedArmSim shoulderSim;
    private final SingleJointedArmSim wristSim; 
   
    public DoubleJointedArmSim(
        DCMotor shoulderDCMotorGearbox, double shoulderGearing, double shoulderMomentOfInertia, double shoulderArmLengthMeters, double shoulderMinAngleRads, double shoulderMaxAngleRads, 
        
        DCMotor wristDCMotorGearbox, double wristGearing, double wristMomentOfIntertia,
        double wristArmLengthMeters, double wristMinAngleRads, double wristMaxAngleRads) {

        shoulderSim = new SingleJointedArmSim(
            shoulderDCMotorGearbox, 
            shoulderGearing, 
            shoulderMomentOfInertia + wristMomentOfIntertia, 
            shoulderArmLengthMeters + wristArmLengthMeters, 
            shoulderMinAngleRads, shoulderMaxAngleRads,
             true);

        wristSim = new SingleJointedArmSim(wristDCMotorGearbox, wristGearing, wristMomentOfIntertia, wristArmLengthMeters, wristMinAngleRads, wristMaxAngleRads, true);
    }

    public double getShoulderAngleDegrees() {
        return Math.toDegrees(shoulderSim.getOutput(0));
    }
    
    // this is world relative
    public double getWristAngleDegrees() {
        return Math.toDegrees(wristSim.getOutput(0));
    }

    public void setInput(double shoulderVoltage, double wristVoltage) {
        shoulderSim.setInput(shoulderVoltage);
        wristSim.setInput(wristVoltage);
    }

    public void update(double dtSeconds) {
        shoulderSim.update(dtSeconds);
        wristSim.update(dtSeconds);
    }
    
}