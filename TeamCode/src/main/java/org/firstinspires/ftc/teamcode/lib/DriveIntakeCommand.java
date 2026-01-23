package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

public class DriveIntakeCommand implements Command{
    public Command.CommandType t = CommandType.DRIVE_INTAKE;
    PathChain path;
    boolean runAlready = false;
    boolean holdEnd = false;
    public AutoSS ss;
    public DriveIntakeCommand(AutoSS ss, PathChain p, boolean holdEnd){
        path = p;
        this.holdEnd = holdEnd;
        this.ss = ss;
    }

    public boolean run() {
        if (!runAlready) {
            ss.follower.followPath(path, holdEnd);
            ss.intakeSubsystem.setIntake(true);
            runAlready = true;
        }
        ss.follower.update();
        if (ss.follower.isBusy()) return true;
        ss.intakeSubsystem.setIntake(false);
        return false;
    }
}
