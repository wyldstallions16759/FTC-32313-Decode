package org.firstinspires.ftc.teamcode;


import android.health.connect.TimeInstantRangeFilter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@Autonomous
public class AUTOBLUEDEPOT extends OpMode {

    public Timer pathTimer;
    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */



    // Setup a variable for each drive wheel to save power level for telemetry








    private ShooterSubsystem shooterSubsystem;
    private AUTOlogic autoLogic;
   private AUTOlogic.PathState pathState;






    @Override


    public void init() {
        autoLogic = new AUTOlogic();
        autoLogic.follower = Constants.createFollower(hardwareMap);
        autoLogic.startPose = new Pose(22.0523882, 123.82126348, Math.toRadians(145));
        autoLogic.ShootingPose = new Pose(52.893682588597834,101.6764252, Math.toRadians(143.5));
        autoLogic.GatherPose1 = new Pose(55.88003521901827,85.10895883777242, Math.toRadians(180));
        autoLogic.GatherPose2 = new Pose(19.27358573629768,84.63174114021572,Math.toRadians(180));
        autoLogic.GatherPose3 = new Pose(58.43583535108961,59.38498789346242, Math.toRadians(180));
        autoLogic.GatherPose4 = new Pose(14.469733656174345,59.38498789346242, Math.toRadians(180));
        autoLogic.GatherPose5 = new Pose(19.469733656174345,59.38498789346242, Math.toRadians(180));
        autoLogic.endPose = new Pose(40.18644067796609,84.87167070217917, Math.toRadians(143.5));
        autoLogic.follower.setPose(autoLogic.startPose);
        autoLogic.buildPaths();

        autoLogic.pathTimer = new Timer();

        autoLogic.intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        autoLogic.leftFeeder = hardwareMap.get(Servo.class, "left_feeder");
        autoLogic.rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
        autoLogic.launcher = hardwareMap.get(DcMotorEx.class, "shooter");


        autoLogic.pathState = AUTOlogic.PathState.DRIVE_STARTPOS_SHOOT_POS;

    //TODO add in any other init mechanisms
    shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);




    }
    public void start() {
        autoLogic.setPathState(autoLogic.pathState);
        autoLogic.pathTimer.resetTimer();
    }
    public void loop() {

        autoLogic.follower.update();
        autoLogic.statePathUpdate();
        telemetry.addData("velocity", autoLogic.launcher.getVelocity());
        telemetry.addData("path state", autoLogic.pathState.toString());
        telemetry.addData("x", autoLogic.follower.getPose().getX());
        telemetry.addData("y", autoLogic.follower.getPose().getY());
        telemetry.addData("heading", autoLogic.follower.getPose().getHeading());
        telemetry.addData("busy", autoLogic.follower.isBusy());
        telemetry.addData("changed", true);
        telemetry.addData("Path timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
    }

