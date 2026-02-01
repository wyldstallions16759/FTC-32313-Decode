package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

public class ShootCommand implements Command{
    public Command.CommandType t = CommandType.SHOOT;
    boolean runAlready = false;
    public int targetVelo;
    public enum ShootState {
        PRIME_SHOOTER,
        SHOOT_BALL,
        STOP_FEED
    }
    public ShootState shootState;
    public AutoSS ss;

    public Timer shootTimer = new Timer();
    public ShootCommand(AutoSS ss, int targetVelo){
        this.ss = ss;
        this.targetVelo = targetVelo;
    }

    public boolean run() {
        //This loop basically just runs a state machine
        if (!runAlready){
            setShootState(ShootState.PRIME_SHOOTER);
            runAlready = true;
        }
        return shootUpdate();
    }
    public void setShootState(ShootState state){
        shootState = state;
        shootTimer.resetTimer();

    }
    public boolean shootUpdate(){
        switch (shootState){
            case PRIME_SHOOTER:
                ss.shooterSubsystem.setShooter(this.targetVelo);
                setShootState(ShootState.SHOOT_BALL);
                break;
            case SHOOT_BALL:
                if (ss.shooterSubsystem.atSpeed()) {
                    ss.shooterSubsystem.feedBall();
                    setShootState(ShootState.STOP_FEED);
                }
                break;
            case STOP_FEED:
                if (ss.shooterSubsystem.atSpeed()){
                    ss.shooterSubsystem.feedBall();
                } else {
                    ss.shooterSubsystem.stopFeed();
                }
                if (shootTimer.getElapsedTimeSeconds() > 4.5){
                    ss.shooterSubsystem.stopFeed();
                    ss.shooterSubsystem.setShooter(0);
                    ss.shooterSubsystem.shooterLoop();
                    return false;
                }
        }
        ss.shooterSubsystem.shooterLoop();
        return true;
    }
}
