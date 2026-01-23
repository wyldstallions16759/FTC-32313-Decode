package org.firstinspires.ftc.teamcode.lib;

public interface Command {
    public CommandType t = CommandType.NONE;
    public enum CommandType {
        NONE,
        DRIVE,
        SHOOT,
        DRIVE_INTAKE
    }
    public boolean run();
}
