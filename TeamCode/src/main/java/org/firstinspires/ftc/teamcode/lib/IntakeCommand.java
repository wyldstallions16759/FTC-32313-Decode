package org.firstinspires.ftc.teamcode.lib;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

public class IntakeCommand implements Command {
    public CommandType t = CommandType.INTAKE;
    AutoSS ss;
    public Timer intakeTimer = new Timer();
    boolean runAlready = false;

    public IntakeCommand(AutoSS ss){
        this.ss = ss;
    }

    @Override
    public boolean run() {
        if (!runAlready){
            ss.intakeSubsystem.setIntake(true);
            intakeTimer.resetTimer();
            runAlready = true;
        }
        if (intakeTimer.getElapsedTimeSeconds() < 2) return true;
        ss.intakeSubsystem.setIntake(false);
        return false;

    }
}
