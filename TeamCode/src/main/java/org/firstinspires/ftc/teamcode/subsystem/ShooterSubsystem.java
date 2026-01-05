package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem {
    DcMotor shooter;
    Servo rightFeeder;
    Servo leftFeeder;
    boolean shooterOn = false;
    Telemetry telemetry;
    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        this.telemetry = telemetry;
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
        leftFeeder = hardwareMap.get(Servo.class, "left_feeder");
    }

    public void startShooter(){
        shooter.setPower(1);
        leftFeeder.setPosition(1);
        rightFeeder.setPosition(0);
        shooterOn = true;
    }
    public void stopShooter(){
        shooter.setPower(0);
        leftFeeder.setPosition(0.5);
        rightFeeder.setPosition(0.5);
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