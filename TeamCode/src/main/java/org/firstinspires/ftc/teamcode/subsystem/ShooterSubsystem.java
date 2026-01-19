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
    private int TARGET_VELOCITY = 1350;
    private int NEW_TARGET_VELOCITY = TARGET_VELOCITY;
    private Servo leftFeeder = null;
    boolean shooterOn = false;
    Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        this.telemetry = telemetry;
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
        leftFeeder = hardwareMap.get(Servo.class, "left_feeder");

    }


    public void startShooter(){
        shooter.setVelocity(1350);
        shooterOn = true;


        telemetry.addLine(" START SHOOTER");
    }
    public void TeleOpstartShooter(){
        shooter.setVelocity(NEW_TARGET_VELOCITY);
    }
    public void speedUp(){
       TARGET_VELOCITY = TARGET_VELOCITY +50;
    }
    public void speedDown(){
        TARGET_VELOCITY = TARGET_VELOCITY - 50;
    }
    public void feedBall(){
        leftFeeder.setPosition(1);
        rightFeeder.setPosition(0);
        intake.setPower(-1);
    }
    public void stopFeed(){
        leftFeeder.setPosition(0.5);
        rightFeeder.setPosition(0.5);
        intake.setPower(-0.5);
    }
    public void stopShooter(){
        shooter.setPower(0);
        shooterOn = false;
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