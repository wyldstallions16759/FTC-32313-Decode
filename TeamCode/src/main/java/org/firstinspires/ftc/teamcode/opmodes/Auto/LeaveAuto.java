package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

@Autonomous(name="Leave Auto")
public class LeaveAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoSS ss = new AutoSS(hardwareMap, telemetry);

        ss.follower.setStartingPose(new Pose(5,0));

        CommandScheduler commandScheduler = new CommandScheduler(ss,
                new DriveCommand(ss, new PathChain(
                        new Path(
                            new BezierLine(
                                    new Pose(5,0),
                                    new Pose(5,12)
                            )
                        )
                ), true)
        );

        waitForStart();

        while (opModeIsActive()){
            boolean res = commandScheduler.commandLoop();
            if (!res) break;
            telemetry.update();
        }
    }
}
