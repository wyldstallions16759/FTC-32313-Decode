package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class AutoSS {
    public Follower follower;
    public ShooterSubsystem shooterSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public Telemetry telemetry;
    public AutoSS(HardwareMap hwmap, Telemetry tele){
        follower = Constants.createFollower(hwmap);
        telemetry = tele;
        shooterSubsystem = new ShooterSubsystem(hwmap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hwmap, telemetry);
    }
}
