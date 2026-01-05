package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystemRR;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystemRR;

@Autonomous(name="RR Auto")
public class RoadRunnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d START_POSE = new Pose2d(-72, 9, Math.toRadians(0));
        Pose2d SHOOT_POSE /*Devondrious*/ = new Pose2d(51, 46, Math.toRadians(45));
        ShooterSubsystemRR shooter = new ShooterSubsystemRR(hardwareMap, telemetry);
        IntakeSubsystemRR intake = new IntakeSubsystemRR(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-72, 9, Math.toRadians(0)));

        waitForStart();
        resetRuntime();

        SequentialAction action = new SequentialAction(
                drive.actionBuilder(START_POSE)
                        .splineToLinearHeading(SHOOT_POSE,0)
                        .build(),
                shooter.StartShooterAction(),
                new SleepAction(5),
                shooter.StopShooterAction()
        );
    }
}
