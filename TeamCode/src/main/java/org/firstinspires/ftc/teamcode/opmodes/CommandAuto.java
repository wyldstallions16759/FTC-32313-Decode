package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.DriveIntakeCommand;
import org.firstinspires.ftc.teamcode.lib.ShootCommand;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.TestConstant;
import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

import java.util.Dictionary;

@Autonomous(name = "Command Auto")
public class CommandAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoSS ss = new AutoSS(hardwareMap, telemetry);
        TestConstant constants = new TestConstant(ss.follower);
        ss.follower.setStartingPose(TestConstant.START_POSE);

        CommandScheduler commandScheduler = new CommandScheduler(ss,
                new DriveIntakeCommand(ss, constants.DRIVETOSHOOT, true),
                new ShootCommand(ss)
        );

        waitForStart();

        while (opModeIsActive()){
            boolean res = commandScheduler.commandLoop();
            if (!res) break;
            telemetry.update();
        }
    }
}
