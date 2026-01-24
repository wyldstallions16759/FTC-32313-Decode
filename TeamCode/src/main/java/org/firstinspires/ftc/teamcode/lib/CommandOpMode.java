package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

public class CommandOpMode extends OpMode {
    CommandScheduler commandScheduler;
    AutoSS ss;
    @Override
    public void init() {
        ss = new AutoSS(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        boolean res = commandScheduler.commandLoop();
        if (!res) stop();
        telemetry.update();
    }
}
