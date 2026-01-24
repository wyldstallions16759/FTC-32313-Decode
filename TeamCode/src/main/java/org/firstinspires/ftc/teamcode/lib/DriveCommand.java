package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

public class DriveCommand implements Command{
    public CommandType t = CommandType.DRIVE;
    PathChain path;
    boolean runAlready = false;
    boolean holdEnd = false;
    public AutoSS ss;
    public DriveCommand(AutoSS ss, PathChain p, boolean holdEnd){
        //We need the path and the holdEnd option for followPath
        path = p;
        this.holdEnd = holdEnd;
        this.ss = ss;
    }

    public boolean run() {
        //On the first time we run it and ONLY the first time, we run follow path
        if (!runAlready) {
            ss.follower.followPath(path, holdEnd);
            runAlready = true;
            ss.follower.update();
            return true;
        }
        ss.follower.update();
        return ss.follower.isBusy();
    }
}
