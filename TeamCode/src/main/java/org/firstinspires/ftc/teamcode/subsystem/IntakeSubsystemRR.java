package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystemRR {

    IntakeSubsystem system;
    public IntakeSubsystemRR(HardwareMap hardwareMap, Telemetry telemetry){
        system = new IntakeSubsystem(hardwareMap, telemetry);

    }
    public class StartIntakeAction implements Action {
        public boolean run(TelemetryPacket packet) {
            system.startIntake();
            return false;
        }

    }
    public Action StartIntakeAction(){return new StartIntakeAction();}
    public class StopIntakeAction implements Action {
        public boolean run(TelemetryPacket packet){
            system.stopIntake();
            return false;
        }
    }
    public Action StopIntakeAction(){return new StopIntakeAction();}
}
