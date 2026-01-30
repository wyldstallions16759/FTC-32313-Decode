package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.DriveCommand;
import org.firstinspires.ftc.teamcode.lib.DriveIntakeCommand;
import org.firstinspires.ftc.teamcode.lib.ShootCommand;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.BlueTriangleAutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.RedCommandAutoConstants;
import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

@Autonomous(name="Blue Triangle Auto")
public class BlueTriangleAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoSS ss = new AutoSS(hardwareMap, telemetry);
        ss.follower.setStartingPose(new Pose(56,8, Math.toRadians(90)));

        BlueTriangleAutoConstants constants = new BlueTriangleAutoConstants(ss.follower);

        CommandScheduler commandScheduler = new CommandScheduler(ss,
                new DriveCommand(ss, constants.TURNTOSHOOT, true),
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
