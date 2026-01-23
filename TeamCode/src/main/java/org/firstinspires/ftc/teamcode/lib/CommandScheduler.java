package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

import java.util.ArrayList;
import java.util.Arrays;

public class CommandScheduler {
    AutoSS ss;
    public ArrayList<Command> commands;
    int i;
    int l;
    public CommandScheduler(AutoSS ss, Command... n){
        this.ss = ss;
        commands = new ArrayList<>();
        commands.addAll(Arrays.asList(n));
        l = commands.size();
    }

    public boolean commandLoop(){
        if (i >= l) return false;
        ss.telemetry.addData("Running Command Index", i);
        ss.telemetry.addData("Command Type", commands.get(i).getClass().getSimpleName());
        boolean result = commands.get(i).run();
        if (!result){
            i++;
        }
        return i < l;
    }
}
