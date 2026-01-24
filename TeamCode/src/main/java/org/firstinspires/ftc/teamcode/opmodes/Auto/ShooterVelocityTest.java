package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@TeleOp(name = "Shooter test")
public class ShooterVelocityTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        while (opModeIsActive()){
            shooter.shooterLoop();
            shooter.feedBall();
        }
    }
}
