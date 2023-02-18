package com.stuypulse.robot.subsystems.intake;

public class NoIntake extends Intake {

    @Override
    public void acquireCube() {
    }

    @Override
    public void acquireCone() {
    }

    @Override
    public void deacquireCube() {
    }

    @Override
    public void deacquireCone() {
    }

    @Override
    public boolean hasNewGamepiece() {
        return false;
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean hasNewGamePiece() {
        return false;
    }
    
}
