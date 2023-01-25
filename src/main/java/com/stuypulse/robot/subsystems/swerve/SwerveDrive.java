package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.subsystems.ISwerveModule;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {


    /** MODULES */
    private final ISwerveModule[] modules;

    /** SENSORS */
    private final AHRS gyro;
    
    private final SwerveDriveKinematics kinematics;
    
    private final Field2d field;
    private final FieldObject2d[] module2ds;

    public SwerveDrive(ISwerveModule... modules) {
        this.modules = modules;

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(getModuleLocations());
        
        field = new Field2d();
        module2ds = new FieldObject2d[modules.length];       
        for(int i = 0; i < module2ds.length; ++i){
            module2ds[i] = field.getObject(modules[i].getID()+"-2d");
        }

        SmartDashboard.putData("Field", field);
        
    }
    
    private Translation2d[] getModuleLocations() {
        Translation2d[] locations = new Translation2d[modules.length];    
        
        for(int i = 0; i < modules.length; ++i) {
            locations[i] = modules[i].getLocation();
        }
        
        return locations;
    }

    private Field2d getField() {
        return field;
    }
    
    private ISwerveModule getModule(String id) {
        for (ISwerveModule module : modules) 
            if (module.getID().equals(id)) {
                return module;
        }
        throw new IllegalArgumentException("Couldn't find the module with id \"" + id + "\"");
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int module = 0; module < modules.length; module++) {
            states[module] = modules[module].getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }
    
    /** MODULE STATES API **/

    public void drive(Vector2D velocity, double omega) {
        // ADD 254 DRIFT FIX!
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.y, -velocity.x,
                -omega,
                Odometry.getInstance().getAngle());

        setChassisSpeeds(speeds, false);
    }
    
    public void setChassisSpeeds(ChassisSpeeds robotSpeed, boolean fieldRelative) {
        if (fieldRelative) {
            robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                robotSpeed,
                Odometry.getInstance().getAngle());
        }

        setModuleStates(kinematics.toSwerveModuleStates(robotSpeed));
    }
    
    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, -1 /* Swerve.MAX_SPEED */);
        
        for(int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds(), true);
    }
    
    /** GYRO API **/
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }
    
    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }
    
    /* Kinematics*/

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    
    @Override
    public void periodic() {
        
    }
    
}