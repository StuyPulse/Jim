package com.stuypulse.robot.util;

import java.util.Optional;

public interface Camera {
    public String getTableName();
 
    public Optional<AprilTagData> getAprilTagData();

    public boolean hasAprilTagData();

    public void updateAprilTagData();
}