package org.firstinspires.ftc.teamcode.opmodes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Shooter test")
public class ShooterFFTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //My new shooter subsystem code wants to know the velocity it should expect if we give it
        //full power. Use this to figure that out.
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        waitForStart();
        while (opModeIsActive()){
            shooter.setPower(1);
            telemetry.addData("Rad/sec", shooter.getVelocity(AngleUnit.RADIANS));
            telemetry.update();
        }
    }
}
