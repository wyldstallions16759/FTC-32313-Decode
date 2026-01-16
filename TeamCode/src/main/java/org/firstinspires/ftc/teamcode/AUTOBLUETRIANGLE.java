package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@Autonomous
public class AUTOBLUETRIANGLE extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private Servo leftFeeder = null;
    private Servo rightFeeder = null;
    private DcMotor intake = null;
    private GoBildaPinpointDriver pinpoint = null;
    private AUTOlogic autoLogic;
    ElapsedTime feederTimer = new ElapsedTime();


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
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }



    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    boolean lastAState = false;
    boolean intakeOn = false;
    boolean lastYstate = false;
    boolean launcherOn = false;

    Timer ShooterTimer = new Timer();



    AUTOlogic.PathState pathState;
    private ShooterSubsystem shooterSubsystem;

    @Override


    public void init() {
        autoLogic.startPose = new Pose(87.3166412, 8.16035634743875, Math.toRadians(90));
        autoLogic.ShootingPose = new Pose(82.5059508,134.1380846325167, Math.toRadians(0));
        autoLogic.endPose = new Pose(81.8644068,60.624697336561766, Math.toRadians(270));

        leftFeeder = hardwareMap.get(Servo.class, "left_feeder");
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
        launcher = hardwareMap.get(DcMotorEx.class, "shooter");
        pathState = AUTOlogic.PathState.TRIANGLE_DRIVE_START_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanisms
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        autoLogic.buildPaths();
        follower.setPose(autoLogic.startPose);


    }
    public void start() {
        autoLogic.setPathState(pathState);
        autoLogic.setPathState(pathState);
        pathTimer.resetTimer();
    }
    public void loop() {

        follower.update();
        autoLogic.statePathUpdate();
        telemetry.addData("velocity", launcher.getVelocity());
        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("changed", true);
        telemetry.addData("Path timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();

    }
}
