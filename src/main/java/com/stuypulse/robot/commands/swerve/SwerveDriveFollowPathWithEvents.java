package com.stuypulse.robot.commands.swerve;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveFollowPathWithEvents extends FollowPathWithEvents {

    private final Map<String, Command> eventMap;
    private final Command pathFollowingCommand;

    public SwerveDriveFollowPathWithEvents(Command pathFollowingCommand,
    List<PathPlannerTrajectory.EventMarker> pathMarkers,
    Map<String, Command> eventMap) {
        super(pathFollowingCommand, pathMarkers, eventMap);
        
        this.eventMap = eventMap;
        this.pathFollowingCommand = pathFollowingCommand;
    }

    @Override
    public boolean isFinished() {
        for (String command : eventMap.keySet()) {
            if (!eventMap.get(command).isFinished()) {
                return false;
            }
        }
        return pathFollowingCommand.isFinished();
    }
}
