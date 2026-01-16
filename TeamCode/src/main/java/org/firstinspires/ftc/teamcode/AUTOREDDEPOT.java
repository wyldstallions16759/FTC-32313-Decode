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
public class AUTOREDDEPOT extends OpMode {

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
    AUTOBLUEDEPOT autoBlueDepot = new AUTOBLUEDEPOT();
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


    AUTOBLUEDEPOT.PathState pathState;





    private ShooterSubsystem shooterSubsystem;



    public void setPathState(AUTOBLUEDEPOT.PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override


    public void init() {
        autoBlueDepot.startPose = new Pose(122.20823244552061, 123.42857142857142, Math.toRadians(37));
        autoBlueDepot.ShootingPose = new Pose(90.583315,101.6764252, Math.toRadians(37));
        autoBlueDepot.GatherPose1 = new Pose(87.5969624,85.10895883777242, Math.toRadians(0));
        autoBlueDepot.GatherPose2 = new Pose(127.979639,84.63174114021572,Math.toRadians(0));
        autoBlueDepot.GatherPose3 = new Pose(85.0411622,59.38498789346242, Math.toRadians(0));
        autoBlueDepot.GatherPose4 = new Pose(129.007264,59.38498789346242, Math.toRadians(0));
        autoBlueDepot.GatherPose5 = new Pose(122.007264,59.38498789346242, Math.toRadians(0));
        leftFeeder = hardwareMap.get(Servo.class, "left_feeder");
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
        launcher = hardwareMap.get(DcMotorEx.class, "shooter");
        pathState = AUTOBLUEDEPOT.PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanisms
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
        autoBlueDepot.buildPaths();
        follower.setPose(autoBlueDepot.startPose);


    }
    public void start() {
        setPathState(pathState);
    }
    public void loop() {

        follower.update();
        autoBlueDepot.statePathUpdate();
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
