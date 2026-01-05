package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystemRR {
    ShooterSubsystem system;
    public ShooterSubsystemRR(HardwareMap hardwareMap, Telemetry telemetry){
        system = new ShooterSubsystem(hardwareMap, telemetry);

    }
    public class StartShooterAction implements Action {
        public boolean run(TelemetryPacket packet) {
            system.startShooter();
            return false;
        }
    }

    //What did run return
    // true --> call the run method
    // false -->

    public Action StartShooterAction(){return new StartShooterAction();}
    public class StopShooterAction implements Action {
        public boolean run(TelemetryPacket packet){
            system.stopShooter();
            return false;
        }
    }
    public Action StopShooterAction(){return new StopShooterAction();}
}
