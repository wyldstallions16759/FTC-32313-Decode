package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.DriveCommand;
import org.firstinspires.ftc.teamcode.lib.DriveIntakeCommand;
import org.firstinspires.ftc.teamcode.lib.IntakeCommand;
import org.firstinspires.ftc.teamcode.lib.ShootCommand;
import org.firstinspires.ftc.teamcode.opmodes.AutoConstants.BlueCommandAutoConstants;
import org.firstinspires.ftc.teamcode.subsystem.AutoSS;

@Autonomous(name = "Blue Command Auto")
public class BlueCommandAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        //This is the command version of the blue depot auto.
        //It is the exact same as the red auto, the only difference is the path config file.

        //This has a shooter, follower, and intake. You need an ss to make commands
        AutoSS ss = new AutoSS(hardwareMap, telemetry);

        //This builds the paths in the constants file
        BlueCommandAutoConstants constants = new BlueCommandAutoConstants(ss.follower);
        ss.follower.setStartingPose(BlueCommandAutoConstants.START_POSE);

        //The command scheduler is a class that runs commands in a sequence
        //You pass in your commands in as parameters. There are only the 3 types for now
        //I commented the scheduler and commands if you want to see them.
        CommandScheduler commandScheduler = new CommandScheduler(ss,
                new DriveCommand(ss, constants.DRIVETOSHOOT, true),
                new ShootCommand(ss, 1525),
                new DriveIntakeCommand(ss, constants.INTAKELINE1, false),
//                new IntakeCommand(ss),
                new DriveIntakeCommand(ss, constants.LINE1TOSHOOT, true),
                new ShootCommand(ss, 1525),
                new DriveIntakeCommand(ss, constants.INTAKELINE2, false),
//                new IntakeCommand(ss),
                new DriveIntakeCommand(ss, constants.LINE2TOSHOOT, true),
                new ShootCommand(ss, 1525),
                new DriveIntakeCommand(ss, constants.INTAKELINE3, false),
//                new IntakeCommand(ss),
                new DriveIntakeCommand(ss, constants.LINE3TOSHOOT, true),
                new ShootCommand(ss, 1525)
        );

        waitForStart();

        while (opModeIsActive()){
            //This progresses the commands. The scheduler handles all of state switching and stuff
            boolean res = commandScheduler.commandLoop();

            //The command loop will return true if it's still busy running commands
            //So we check if it's done by asking if it's false. If it is we break out of the loop.
            if (!res) break;
            telemetry.update();
        }
    }
}
