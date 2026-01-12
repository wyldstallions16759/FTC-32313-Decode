package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem {
    private DcMotorEx shooter = null;
    private DcMotor intake = null;

    private Servo rightFeeder = null;
    private Servo leftFeeder = null;
    boolean shooterOn = false;
    Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();
    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
    shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    intake = hardwareMap.get(DcMotor.class, "intake");
        this.telemetry = telemetry;
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
        leftFeeder = hardwareMap.get(Servo.class, "left_feeder");

    }


    public void startShooter(){
        timer.reset();
        shooter.setVelocity(1250);
        shooterOn = true;
        while (shooterOn) {
            if (timer.milliseconds() > 3250) {
                timer.reset();
                intake.setPower(-0.5);
                leftFeeder.setPosition(0.5);
                rightFeeder.setPosition(0.5);

            }
            if (timer.milliseconds() > 1750) {
                shooter.setVelocity(1300);
                leftFeeder.setPosition(1);
                rightFeeder.setPosition(0);
                intake.setPower(-1);
            }
            }


        telemetry.addLine("CALLED START SHOOTER");
    }
    public void stopShooter(){
        timer.reset();
        shooter.setPower(0);
        leftFeeder.setPosition(0.5);
        rightFeeder.setPosition(0.5);
        intake.setPower(0);
        shooterOn = false;
    }
    public void toggleShooter(){
        if (shooterOn){
            stopShooter();
        }else{
            startShooter();
        }
    }
    public void setShooter(boolean on){
        if (on){
            startShooter();
        } else {
            stopShooter();
        }
    }
    //method that turns the shooter on if its off and off if its on
}
// object.methodName(8, 10);
// if (condition){}