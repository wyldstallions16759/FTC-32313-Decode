package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

public class TurnCommand implements Command{
    public CommandType t = CommandType.TURN;
    double heading;
    boolean runAlready = false;
    public AutoSS ss;
    public TurnCommand(AutoSS ss, double heading){
        this.heading = heading;
        this.ss = ss;
    }

    public boolean run() {
        if (!runAlready) {
            ss.follower.turnToDegrees(heading);
            runAlready = true;
            ss.follower.update();
            return true;
        }
        ss.follower.update();
        ss.telemetry.addData("Current position", ss.follower.getPose());
        return ss.follower.isBusy();
    }
}
