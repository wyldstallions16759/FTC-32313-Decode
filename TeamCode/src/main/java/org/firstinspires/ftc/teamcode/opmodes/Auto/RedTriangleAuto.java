package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.DriveCommand;
import org.firstinspires.ftc.teamcode.lib.DriveIntakeCommand;
import org.firstinspires.ftc.teamcode.lib.ShootCommand;
import org.firstinspires.ftc.teamcode.lib.TurnCommand;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.BlueTriangleAutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.RedTriangleAutoConstants;
import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

@Autonomous(name="Red Triangle Auto")
public class RedTriangleAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoSS ss = new AutoSS(hardwareMap, telemetry);
        ss.follower.setStartingPose(new Pose(88,8, Math.toRadians(90)));

        RedTriangleAutoConstants constants = new RedTriangleAutoConstants(ss.follower);
        CommandScheduler commandScheduler = new CommandScheduler(ss,
                new TurnCommand(ss, 67),
                new ShootCommand(ss, 1800),
                new DriveIntakeCommand(ss, constants.INTAKELINE1, false),
                new DriveCommand(ss, constants.LINE1TOSHOOT, true),
                new ShootCommand(ss, 1800),
                new DriveCommand(ss, constants.LEAVE, true)
        );

        waitForStart();
        while (opModeIsActive()){
            boolean res = commandScheduler.commandLoop();
            if (!res) break;
            telemetry.update();
        }
    }
}
