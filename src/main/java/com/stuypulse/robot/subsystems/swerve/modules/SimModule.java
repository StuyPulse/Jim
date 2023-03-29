package com.stuypulse.robot.subsystems.swerve.modules;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.Robot.MatchState;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.ARateLimit;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
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
    private Controller driveController;
    private AngleController turnController;

    public SimModule(String id, Translation2d location) {
        
        // module data
        this.id = id;
        this.location = location;

        // turn 
        turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Turn.kV.get(), Turn.kA.get()));

        turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setSetpointFilter(new ARateLimit(Swerve.MAX_TURNING));

        // drive
        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(Drive.kV, Drive.kA));
        
        driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
                .setOutputFilter(x -> Robot.getMatchState() == MatchState.TELEOP ? 0 : x)
            .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());
        
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
        turnController.update(
            Angle.fromRotation2d(targetState.angle), 
            Angle.fromRotation2d(getAngle()));

        // drive
        driveController.update(
            targetState.speedMetersPerSecond, 
            getVelocity());

        SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", targetState.angle.getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Error", turnController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Voltage", turnController.getOutput());
        SmartDashboard.putNumber("Swerve/" + id + "/Angle Current", turnSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Swerve/" + id + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Error", driveController.getError());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Voltage", driveController.getOutput());
        SmartDashboard.putNumber("Swerve/" + id + "/Velocity Current", driveSim.getCurrentDrawAmps());
    }

    @Override
    public void simulationPeriodic() {
        // drive
        driveSim.setInput(driveController.getOutput());
        driveSim.update(Settings.DT);
        
        // turn
        turnSim.setInput(turnController.getOutput());
        turnSim.update(Settings.DT);
        
       // turn simulation
       RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
           turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
       ));

    }
}
