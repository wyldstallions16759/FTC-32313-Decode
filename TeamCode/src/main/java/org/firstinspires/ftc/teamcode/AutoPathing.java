package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@Autonomous
public class AutoPathing extends OpMode {

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
    public enum PathState {
        //
        // START POSITION - END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        PRIME_SHOOTER_1,
        SHOOT_BALL_1,
        STOP_FEED_1,
        START_FEED_2,
        STOP_FEED_2,
        START_FEED_3,
        STOP_FEED_3,
        PRIME_SHOOTER_4,
        SHOOT_BALL_4,
        STOP_FEED_4,
        START_FEED_5,
        STOP_FEED_5,
        START_FEED_6,
        STOP_FEED_6,
        DRIVE_SHOOT_GATHERPOSE_1,
        DRIVE_GATHERPOSE_1_GATHERPOSE_2,
        DRIVE_GATHERPOSE_2_SHOOT,




        DRIVE_SHOOT_END

    }

    PathState pathState;


    private final Pose startPose = new Pose(22.0523882, 123.82126348, Math.toRadians(145));
    private final Pose ShootingPose = new Pose(52.893682588597834,101.6764252, Math.toRadians(145));
    private final Pose GatherPose1 = new Pose(55.88003521901827,85.10895883777242, Math.toRadians(180));
    private final Pose GatherPose2 = new Pose(15.497358573629768,84.63174114021572,Math.toRadians(180));        ;

    private PathChain driveStartPosShootPos, driveShootPosGatherPos1, driveGatherPos1GatherPos2, driveGatherPos2ShootPos;

    private ShooterSubsystem shooterSubsystem;
    public void buildPaths() {
        // put in start and end pos
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootingPose.getHeading())
                .build();
        driveShootPosGatherPos1 = follower.pathBuilder()
                .addPath(new BezierLine(ShootingPose,GatherPose1))
                .setLinearHeadingInterpolation(ShootingPose.getHeading(),GatherPose1.getHeading())
                .build();
        driveGatherPos1GatherPos2 = follower.pathBuilder()
                .addPath(new BezierLine(GatherPose1,GatherPose2))
                .setLinearHeadingInterpolation(GatherPose1.getHeading(), GatherPose2.getHeading())
                .build();
        driveGatherPos2ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(GatherPose2,ShootingPose))
                .setLinearHeadingInterpolation(GatherPose2.getHeading(), ShootingPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.PRIME_SHOOTER_1); /*it goes to the public void down there and changes
                 the variable pathState and resets the timer that we want to reset every time we set a new path */
                break;
            case PRIME_SHOOTER_1:
                if (!follower.isBusy()) {

                    shooterSubsystem.startShooter();
                }
                setPathState(PathState.SHOOT_BALL_1);
                break;

            case SHOOT_BALL_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooterSubsystem.feedBall();
                }
                setPathState(PathState.STOP_FEED_1);
                break;
            case STOP_FEED_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.75) {
                    shooterSubsystem.stopFeed();
                }
                setPathState(PathState.START_FEED_2);
                break;
            case START_FEED_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    shooterSubsystem.feedBall();
                }
                setPathState(PathState.STOP_FEED_2);
                break;
            case STOP_FEED_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5.25) {
                    shooterSubsystem.stopFeed();
                }
                setPathState(PathState.START_FEED_3);
                break;
            case START_FEED_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5.75) {
                    shooterSubsystem.feedBall();
                }
                setPathState(PathState.STOP_FEED_3);
                break;
            case STOP_FEED_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 7.5) {
                    shooterSubsystem.stopFeed();
                    shooterSubsystem.stopShooter();
                }
                setPathState(PathState.DRIVE_SHOOT_GATHERPOSE_1);
                //check is follower done it's path?

                break;
            case DRIVE_SHOOT_GATHERPOSE_1:
                pathTimer.resetTimer();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(driveShootPosGatherPos1);
                    telemetry.addLine("Done Gather Pose 1");

                }
                setPathState(PathState.DRIVE_GATHERPOSE_1_GATHERPOSE_2);
                break;
            case DRIVE_GATHERPOSE_1_GATHERPOSE_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(driveGatherPos1GatherPos2);
                    telemetry.addLine("Done Gather Pose 2");
                }
                setPathState(PathState.DRIVE_GATHERPOSE_2_SHOOT);
                    break;
            case DRIVE_GATHERPOSE_2_SHOOT:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 6) {
                    follower.followPath(driveGatherPos2ShootPos);
                    telemetry.addLine("Done Shoot Pos 2");
                }

                setPathState(PathState.PRIME_SHOOTER_4);
                break;
            case PRIME_SHOOTER_4:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    shooterSubsystem.startShooter();
                }
                setPathState(PathState.SHOOT_BALL_4);
                break;

            case SHOOT_BALL_4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooterSubsystem.feedBall();
                }
                setPathState(PathState.STOP_FEED_3);
                break;
            case STOP_FEED_4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.75) {
                    shooterSubsystem.stopFeed();
                }
                setPathState(PathState.START_FEED_5);
                break;
            case START_FEED_5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    shooterSubsystem.feedBall();
                }
                setPathState(PathState.STOP_FEED_5);
                break;
            case STOP_FEED_5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5.25) {
                    shooterSubsystem.stopFeed();
                }
                setPathState(PathState.START_FEED_6);
                break;
            case START_FEED_6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5.75) {
                    shooterSubsystem.feedBall();
                }
                setPathState(PathState.STOP_FEED_6);
                break;
            case STOP_FEED_6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 7.5) {
                    shooterSubsystem.stopFeed();
                    shooterSubsystem.stopShooter();
                }

                break;
            default:
                telemetry.addLine("No State Commanded");
                break;

        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    @Override


    public void init() {
        leftFeeder = hardwareMap.get(Servo.class, "left_feeder");
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
    pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
    pathTimer = new Timer();
    opModeTimer = new Timer();
    follower = Constants.createFollower(hardwareMap);
    //TODO add in any other init mechanisms
    shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
    buildPaths();
    follower.setPose(startPose);


    }
    public void start() {
        setPathState(pathState);
    }
    public void loop() {

    follower.update();
    statePathUpdate();

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
