package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.subsystems.swerve.modules.SwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.MAX_SwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SL_SwerveModule;
import com.stuypulse.robot.subsystems.swerve.modules.SacrodModule;
import com.stuypulse.robot.subsystems.swerve.modules.SimModule;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings.Robot;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private static final SwerveDrive instance; 

    static {
        if (RobotBase.isReal()) {
            if (Settings.ROBOT == Robot.JIM) {
                // instance = new SwerveDrive(
                //     new MAX_SwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE),
                //     new MAX_SwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE),
                //     new MAX_SwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE),
                //     new MAX_SwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE)
                // );
                instance = new SwerveDrive(
                    new SL_SwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE),
                    new SL_SwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE),
                    new SL_SwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE),
                    new SL_SwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE)
                );
            } else {
                instance = new SwerveDrive(
                    SacrodModule.createFrontRight(),
                    SacrodModule.createFrontLeft(),
                    SacrodModule.createBackLeft(),
                    SacrodModule.createBackRight()
                );
            }
        } else {
            instance = new SwerveDrive(
                new SimModule(FrontRight.ID, FrontRight.MODULE_OFFSET),
                new SimModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET),
                new SimModule(BackLeft.ID, BackLeft.MODULE_OFFSET),
                new SimModule(BackRight.ID, BackRight.MODULE_OFFSET)
            );
        }
    }
    
    public static SwerveDrive getInstance() {
        return instance;
    }

    /** MODULES */
    private final SwerveModule[] modules;

    /** SENSORS */
    private final AHRS gyro;
    
    private final SwerveDriveKinematics kinematics;

    private final FieldObject2d[] module2ds;

    public SwerveDrive(SwerveModule... modules) {
        this.modules = modules;

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(getModuleOffsets());

        module2ds = new FieldObject2d[modules.length];
    }

    public void initFieldObjects(Field2d field) {
        for (int i = 0; i < modules.length; i++) {
            module2ds[i] = field.getObject(modules[i].getID()+"-2d");
        }
    }
    
    private Translation2d[] getModuleOffsets() {
        Translation2d[] locations = new Translation2d[modules.length];    
        
        for(int i = 0; i < modules.length; ++i) {
            locations[i] = modules[i].getOffset();
        }
        
        return locations;
    }
    
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    private SwerveModule getModule(String id) {
        for (SwerveModule module : modules) 
            if (module.getID().equals(id)) {
                return module;
        }
        throw new IllegalArgumentException("Couldn't find the module with id \"" + id + "\"");
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }
    


    /** MODULE STATES API **/
    public void drive(Vector2D velocity, double omega) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity.y, -velocity.x,
                -omega,
                Odometry.getInstance().getRotation());

        Pose2d robotVel = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        ));
    }
    
    public void setChassisSpeeds(ChassisSpeeds robotSpeed) {
        setModuleStates(kinematics.toSwerveModuleStates(robotSpeed));
    }
    
    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException("Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_SPEED);
        
        for(int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(states[i]);
        }
    }

    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
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

    /** KINEMATICS **/
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getBalanceAngle() {
        Rotation2d pitch = getGyroPitch();
        Rotation2d roll = getGyroRoll();
        Rotation2d yaw = Odometry.getInstance().getRotation();
        
        double facingSlope = pitch.getTan() * yaw.getCos() + roll.getTan() * yaw.getSin();
        double maxSlope = Math.sqrt(Math.pow(roll.getTan(), 2) + Math.pow(pitch.getTan(), 2));
        

        SmartDashboard.putNumber("Swerve/Max Slope", maxSlope);
        SmartDashboard.putNumber("Swerve/Facing Slope", facingSlope);

        return Rotation2d.fromRadians(Math.signum(facingSlope) * Math.atan(maxSlope));
    }

    @Override
    public void periodic() {
        if (Settings.isDebug()) {
            Odometry odometry = Odometry.getInstance();
            Pose2d pose = odometry.getPose();
            Rotation2d angle = odometry.getRotation();

            for (int i = 0; i < modules.length; ++i) {
                module2ds[i].setPose(new Pose2d(
                    pose.getTranslation().plus(modules[i].getOffset().rotateBy(angle)),
                    modules[i].getState().angle.plus(angle)
                ));
            }

            Settings.putNumber("Swerve/Balance Angle (deg)", getBalanceAngle().getDegrees());
            Settings.putNumber("Swerve/Gyro Angle (deg)", getGyroAngle().getDegrees());
            Settings.putNumber("Swerve/Gyro Pitch", getGyroPitch().getDegrees());
            Settings.putNumber("Swerve/Gyro Roll", getGyroRoll().getDegrees());
        }
    }
    
    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.DT));
    }
    
}