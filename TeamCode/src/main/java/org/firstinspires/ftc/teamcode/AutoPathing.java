package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
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
    public enum PathState {
        //
        // START POSITION - END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        DRIVE_SHOOT_END

    }

    PathState pathState;


    private final Pose startPose = new Pose(22.0523882, 123.82126348, Math.toRadians(145));
    private final Pose ShootingPose = new Pose(52.893682588597834,101.6764252, Math.toRadians(145));
    private final Pose GatherPose1 = new Pose(55.705701078582436,87.89830508474577, Math.toRadians(180));
    private final Pose GatherPose2 = new Pose(16.369029275808945,84.63174114021572,Math.toRadians(180));        ;

    private PathChain driveStartPosShootPos, driveShootPosGatherPos1, driveGatherPos1GatherPos2;

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
    }

    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); /*it goes to the public void down there and changes
                 the variable pathState and resets the timer that we want to reset every time we set a new path */
                break;
            case SHOOT_PRELOAD:
                //TODO add logic to flywheel shooter

                //check is follower done it's path?
                if (!follower.isBusy()) {
                    shooterSubsystem.startShooter();
                    telemetry.addLine("Done Path 1!");
                    follower.followPath(driveStartPosShootPos, true);
                    setPathState(PathState.DRIVE_SHOOT_END);
                }
                break;
            case DRIVE_SHOOT_END:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 7) {
                    follower.followPath(driveShootPosGatherPos1);
                    telemetry.addLine("Done all paths");
                    shooterSubsystem.stopShooter();
                    break;

                }
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
        opModeTimer.resetTimer();
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
