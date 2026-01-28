package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {
    DcMotor  intake;
    boolean intakeOn = false;
    Telemetry telemetry;
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        this.telemetry = telemetry;
    }
    public void startIntake(){
        intake.setPower(-1);
        intakeOn = true;
    }
    public void stopIntake(){
        intake.setPower(0);
        intakeOn = false;
    }
    public void setIntake(boolean on){
        if (on) {
            startIntake();
        }else{
            stopIntake();
        }
    }
    public void toggleIntake(){
        if (intakeOn) {
            stopIntake();
        } else{
            stopIntake();
        }
    }
}
