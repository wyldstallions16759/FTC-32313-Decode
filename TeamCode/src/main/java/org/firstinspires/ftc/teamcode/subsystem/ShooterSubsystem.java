package org.firstinspires.ftc.teamcode.subsystem;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.PIDController;

public class ShooterSubsystem {
    private DcMotorEx shooter = null;
    private DcMotor intake = null;

    private Servo rightFeeder = null;
    private double targetVelo = 0;
    private Servo leftFeeder = null;
    boolean shooterOn = false;
    Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();

    private PIDController pid;
    private static final double w_max = 5.6; //TODO: Tune by setting power to 1 and reading the velocity in radians
    private static final double kV = 1.0/w_max;
    private static final double threshold = Math.PI;
    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        this.telemetry = telemetry;
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
        leftFeeder = hardwareMap.get(Servo.class, "left_feeder");
        this.pid = new PIDController(0.001, 0, 0);
    }


    public void startShooter(){
        shooter.setVelocity(1350);
        shooterOn = true;


        telemetry.addLine(" START SHOOTER");
    }
    public void speedUp(){
       targetVelo += 50;
    }
    public void speedDown(){ targetVelo -= 50; }
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

    public void setShooter(double velocity){
        targetVelo = velocity;
    }

    public void shooterLoop(){
        if (targetVelo == 0){
            shooter.setPower(0);
            return;
        }
        pid.setSetPoint(targetVelo);
        double pidCalc = pid.calculate(shooter.getVelocity(AngleUnit.RADIANS));
        double ffCalc = (targetVelo * kV);
        double power = ffCalc + pidCalc;
        if (power < -1.0) power = -1;
        if (power > 1.0) power = 1.0;
        telemetry.addData("Power", power);
        telemetry.addData("FF Calc", ffCalc);
        telemetry.addData("PID Calc", pidCalc);
        telemetry.addData("kV", kV);
        shooter.setPower(power);
        telemetry.addData("Shooter velocity", shooter.getVelocity(AngleUnit.RADIANS));
    }

    public boolean atSpeed(){
        double error = targetVelo - shooter.getVelocity(AngleUnit.RADIANS);
        return (error < threshold && error > -threshold);
    }
}