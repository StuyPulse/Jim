package com.stuypulse.robot.commands.swerve;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveFollowTrajectory extends PPSwerveControllerCommand {

	public static HashMap<String, PathPlannerTrajectory> getSeparatedPaths(List<PathPlannerTrajectory> paths, String... names) {
		if (paths.size() != names.length)
			throw new IllegalArgumentException("Invalid number of path names given to FollowTrajectory.getSeparatedPaths");

		HashMap<String, PathPlannerTrajectory> map = new HashMap<String, PathPlannerTrajectory>();

		for (int i = 0; i < names.length; i++) {
			map.put(names[i], paths.get(i));
		}

		return map;
	}

	private boolean robotRelative;
	private PathPlannerTrajectory path;

	public SwerveDriveFollowTrajectory(PathPlannerTrajectory path) {

		super(
			path,
			Odometry.getInstance()::getPose,
			SwerveDrive.getInstance().getKinematics(),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.THETA.kP, Motion.THETA.kI, Motion.THETA.kD),
			SwerveDrive.getInstance()::setModuleStates,
			true,
			SwerveDrive.getInstance()
		);
		
		robotRelative = false;
		this.path = path;
	}

	public SwerveDriveFollowTrajectory robotRelative() {
		robotRelative = true;
		return this;
	}

	public SwerveDriveFollowTrajectory fieldRelative() {
		robotRelative = false;
		return this;
	}

	public FollowPathWithEvents withEvents(Map<String, Command> events) {
		return new FollowPathWithEvents(
			this,
			path.getMarkers(),
			new HashMap<String, Command>(events)
		);
	}

	public FollowPathWithEvents withEvents(HashMap<String, Command> events) {
		return new FollowPathWithEvents(
			this,
			path.getMarkers(),
			events
		);
	}

	@Override
	public void initialize() {
		if (robotRelative) {
			PathPlannerState initialState = PathPlannerTrajectory.transformStateForAlliance(
				path.getInitialState(),
				DriverStation.getAlliance());
			
			Odometry.getInstance().reset(new Pose2d(
				initialState.poseMeters.getTranslation(),
				initialState.holonomicRotation
			));
		}

		super.initialize();
	}

}