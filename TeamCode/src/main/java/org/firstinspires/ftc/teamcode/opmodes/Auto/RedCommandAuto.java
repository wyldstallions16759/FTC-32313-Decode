package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.DriveCommand;
import org.firstinspires.ftc.teamcode.lib.DriveIntakeCommand;
import org.firstinspires.ftc.teamcode.lib.ShootCommand;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.RedCommandAutoConstants;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.BlueCommandAutoConstants;
import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

@Autonomous(name = "Flipped Command Auto")
public class RedCommandAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoSS ss = new AutoSS(hardwareMap, telemetry);
        RedCommandAutoConstants constants = new RedCommandAutoConstants(ss.follower);
        ss.follower.setStartingPose(BlueCommandAutoConstants.START_POSE);

        CommandScheduler commandScheduler = new CommandScheduler(ss,
                new DriveCommand(ss, constants.DRIVETOSHOOT, true),
                new ShootCommand(ss),
                new DriveIntakeCommand(ss, constants.INTAKELINE1, false),
                new DriveIntakeCommand(ss, constants.LINE1TOSHOOT, true),
                new ShootCommand(ss),
                new DriveIntakeCommand(ss, constants.INTAKELINE2, false),
                new DriveIntakeCommand(ss, constants.LINE2TOSHOOT, true),
                new ShootCommand(ss),
                new DriveIntakeCommand(ss, constants.INTAKELINE3, false),
                new DriveIntakeCommand(ss, constants.LINE3TOSHOOT, true),
                new ShootCommand(ss),
                new DriveCommand(ss, constants.INTAKELINE3, true)
        );

        waitForStart();

        while (opModeIsActive()){
            boolean res = commandScheduler.commandLoop();
            if (!res) break;
            telemetry.update();
        }
    }
}