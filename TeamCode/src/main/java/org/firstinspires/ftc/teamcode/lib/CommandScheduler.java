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
        //Takes all of the commands you pass in and throws them into an arraylist
        this.ss = ss;
        commands = new ArrayList<>();
        commands.addAll(Arrays.asList(n));
        l = commands.size();
    }

    public boolean commandLoop(){
        //This loops through the array list
        if (i >= l) return false;
        ss.telemetry.addData("Running Command Index", i);
        ss.telemetry.addData("Command Type", commands.get(i).getClass().getSimpleName());

        //The run method of commands true if it's busy and false if not. Move to the next command
        //if it's not busy
        boolean result = commands.get(i).run();
        if (!result){
            i++;
        }
        return i < l;
    }
}
