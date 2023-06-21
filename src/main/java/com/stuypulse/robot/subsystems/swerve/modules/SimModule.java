/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve.modules;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class SimModule extends SwerveModule {

    private static LinearSystem<N2, N1, N2> identifyVelocityPositionSystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("Kv must be greater than zero.");
          }
          if (kA <= 0.0) {
            throw new IllegalArgumentException("Ka must be greater than zero.");
          }

          return new LinearSystem<N2, N1, N2>(
              Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kV / kA),
              Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 1.0 / kA),
              Matrix.mat(Nat.N2(), Nat.N2()).fill(1.0, 0.0, 0.0, 1.0),
              Matrix.mat(Nat.N2(), Nat.N1()).fill(0.0, 0.0));
    }

    // module data
    private final String id;
    private final Translation2d location;
    private SwerveModuleState targetState;

    // turn
    private final LinearSystemSim<N2, N1, N1> turnSim;

    // drive
    private final LinearSystemSim<N2, N1, N2> driveSim;

    // controllers
    private final PIDController turnPID;
    private final SlewRateLimiter turnRateLimit;
    private double turnVoltage;

    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFF;
    private double driveVoltage;

    public SimModule(String id, Translation2d location) {

        // module data
        this.id = id;
        this.location = location;

        // turn
        turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Turn.kV.get(), Turn.kA.get()));

        turnPID = new PIDController(Turn.kP.get(), Turn.kI, Turn.kD.get());
        turnPID.enableContinuousInput(-Math.PI, Math.PI);

        turnRateLimit = new SlewRateLimiter(Swerve.MAX_TURNING.get());
    
        // drive
        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(Drive.kV, Drive.kA));

        drivePID = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        driveFF = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

        targetState = new SwerveModuleState();
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
        return driveSim.getOutput(1);
    }

    private double getDistance() {
        return driveSim.getOutput(0);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnSim.getOutput(0));
    }

    @Override
    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    @Override
    public void periodic() {
        
        // turn
        turnVoltage = turnPID.calculate(
            getAngle().getRadians(),
            turnRateLimit.calculate(targetState.angle.getRadians()));

        // drive
        driveVoltage = drivePID.calculate(getVelocity(), targetState.speedMetersPerSecond);
        if (Robot.getMatchState() == MatchState.TELEOP) {
            driveVoltage = 0;
        }

        driveVoltage += driveFF.calculate(targetState.speedMetersPerSecond);

        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Error", targetState.angle.minus(getAngle()).getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Voltage", turnVoltage);
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Current", turnSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Swerve/" + id + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Error", targetState.speedMetersPerSecond - getVelocity());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Voltage", driveVoltage);
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Current", driveSim.getCurrentDrawAmps());
    }

    @Override
    public void simulationPeriodic() {
        // drive
        driveSim.setInput(driveVoltage);
        driveSim.update(Settings.DT);

        // turn
        turnSim.setInput(turnVoltage);
        turnSim.update(Settings.DT);

       // turn simulation
       RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
           turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
       ));

    }
}
