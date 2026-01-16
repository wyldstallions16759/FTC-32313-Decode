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
public class AUTOBLUEDEPOT extends OpMode {

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
        PRIME_SHOOTER_2,
        PRIME_SHOOTER_3,
        SHOOT_BALL_1,
        SHOOT_BALL_2,
        SHOOT_BALL_3,
        STOP_FEED_1,
        STOP_FEED_2,
        STOP_FEED_3,

        DRIVE_SHOOT_GATHERPOSE_1,
        DRIVE_GATHERPOSE_1_GATHERPOSE_2,
        DRIVE_GATHERPOSE_2_SHOOT,
        DRIVE_SHOOT_GATHERPOSE_3,
        DRIVE_GATHERPOSE_3_GATHERPOSE_4,
        DRIVE_GATHERPOSE_4_GATHERPOSE_5,
        DRIVE_GATHERPOSE_5_SHOOT,
        DRIVE_SHOOT_ENDPOS,
        TRIANGLE_DRIVE_SHOOT_ENDPOS,
        TRIANGLE_DRIVE_START_SHOOTPOS;

    }

    PathState pathState;


    public Pose startPose = new Pose(22.0523882, 123.82126348, Math.toRadians(145));
    public Pose ShootingPose = new Pose(52.893682588597834,101.6764252, Math.toRadians(143.5));
    public Pose GatherPose1 = new Pose(55.88003521901827,85.10895883777242, Math.toRadians(180));
    public Pose GatherPose2 = new Pose(19.27358573629768,84.63174114021572,Math.toRadians(180));
    public Pose GatherPose3 = new Pose(58.43583535108961,59.38498789346242, Math.toRadians(180));
    public Pose GatherPose4 = new Pose(14.469733656174345,59.38498789346242, Math.toRadians(180));
    public Pose GatherPose5 = new Pose(19.469733656174345,59.38498789346242, Math.toRadians(180));
    public Pose endPose = new Pose(40.18644067796609,84.87167070217917, Math.toRadians(143.5));

    public PathChain
            driveStartPosShootPos,
            driveShootPosGatherPos1,
            driveGatherPos1GatherPos2,
            driveGatherPos2ShootPos,
            driveShootPosGatherPos3,
            driveGatherPos3GatherPos4,
            driveGatherPos4GatherPos5,
            driveGatherPos5ShootPos,
            driveShootPosEndPos;

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
        driveShootPosGatherPos3 = follower.pathBuilder()
                .addPath(new BezierLine(ShootingPose,GatherPose3))
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), GatherPose3.getHeading())
                .build();
        driveGatherPos3GatherPos4 = follower.pathBuilder()
                .addPath(new BezierLine(GatherPose3,GatherPose4))
                .setLinearHeadingInterpolation(GatherPose3.getHeading(), GatherPose4.getHeading())
                .build();
        driveGatherPos4GatherPos5 = follower.pathBuilder()
                .addPath(new BezierLine(GatherPose4,GatherPose5))
                .setLinearHeadingInterpolation(GatherPose4.getHeading(), GatherPose5.getHeading())
                .build();
        driveGatherPos5ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(GatherPose5,ShootingPose))
                .setLinearHeadingInterpolation(GatherPose5.getHeading(), ShootingPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(ShootingPose,endPose))
                .setLinearHeadingInterpolation(ShootingPose.getHeading(), endPose.getHeading())
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
                    pathTimer.resetTimer();
                    shooterSubsystem.startShooter();
                    setPathState(PathState.SHOOT_BALL_1);
                }

                break;

            case SHOOT_BALL_1:
                if (launcher.getVelocity() > 1375) {
                    shooterSubsystem.feedBall();
                    setPathState(PathState.STOP_FEED_1);
                }

                break;
            case STOP_FEED_1:
                if (pathTimer.getElapsedTimeSeconds() > 7.5) {
                    shooterSubsystem.stopFeed();
                    shooterSubsystem.stopShooter();
                    setPathState(PathState.DRIVE_SHOOT_GATHERPOSE_1);
                }

                break;

            case DRIVE_SHOOT_GATHERPOSE_1:

                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosGatherPos1, true);
                    telemetry.addLine("Done Gather Pose 1");
                    setPathState(PathState.DRIVE_GATHERPOSE_1_GATHERPOSE_2);
                    pathTimer.resetTimer();
                }

                break;
            case DRIVE_GATHERPOSE_1_GATHERPOSE_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(driveGatherPos1GatherPos2, true);
                    intake.setPower(-1);
                    telemetry.addLine("Done Gather Pose 2");
                    setPathState(PathState.DRIVE_GATHERPOSE_2_SHOOT);
                    pathTimer.resetTimer();
                }

                    break;
            case DRIVE_GATHERPOSE_2_SHOOT:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(driveGatherPos2ShootPos, true);
                    intake.setPower(-0.5);
                    telemetry.addLine("Done Shoot Pos 2");
                    setPathState(PathState.PRIME_SHOOTER_2);
                }


                break;
            case PRIME_SHOOTER_2:
                    pathTimer.resetTimer();
                    shooterSubsystem.startShooter();
                setPathState(PathState.SHOOT_BALL_2);
                break;

            case SHOOT_BALL_2:
                if (launcher.getVelocity() > 1375) {
                    pathTimer.resetTimer();
                    shooterSubsystem.feedBall();
                    setPathState(PathState.STOP_FEED_2);
                }

                break;
            case STOP_FEED_2:
                if (pathTimer.getElapsedTimeSeconds() > 7.5) {
                    shooterSubsystem.stopFeed();
                    setPathState(PathState.DRIVE_SHOOT_GATHERPOSE_3);
                }

                break;

            case DRIVE_SHOOT_GATHERPOSE_3:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosGatherPos3);
                    setPathState(PathState.DRIVE_GATHERPOSE_3_GATHERPOSE_4);
                }
                break;
            case DRIVE_GATHERPOSE_3_GATHERPOSE_4:
                if (!follower.isBusy()) {
                    follower.followPath(driveGatherPos3GatherPos4);
                    setPathState(PathState.DRIVE_GATHERPOSE_4_GATHERPOSE_5);
                }
                break;
            case DRIVE_GATHERPOSE_4_GATHERPOSE_5:
                if (!follower.isBusy()) {
                    follower.followPath(driveGatherPos4GatherPos5);
                    setPathState(PathState.DRIVE_GATHERPOSE_5_SHOOT);
                }
                break;
            case DRIVE_GATHERPOSE_5_SHOOT:
                if (!follower.isBusy()) {
                    follower.followPath(driveGatherPos5ShootPos);
                    setPathState(PathState.PRIME_SHOOTER_3);
                }
                break;
            case PRIME_SHOOTER_3:
                    pathTimer.resetTimer();
                    shooterSubsystem.startShooter();
                    setPathState(PathState.SHOOT_BALL_3);
                break;
            case SHOOT_BALL_3:
                if (launcher.getVelocity() > 1375){
                    shooterSubsystem.feedBall();
                    setPathState(PathState.STOP_FEED_3);
                }
                break;
            case STOP_FEED_3:
                if(pathTimer.getElapsedTimeSeconds() > 10){
                    shooterSubsystem.stopFeed();
                    setPathState(PathState.DRIVE_SHOOT_ENDPOS);
                }
                break;
            case DRIVE_SHOOT_ENDPOS:
                if (!follower.isBusy()){
                    follower.followPath(driveShootPosEndPos, true);
                }
                break;
            case TRIANGLE_DRIVE_START_SHOOTPOS:
                follower.followPath(driveStartPosShootPos);
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.25){

                    shooterSubsystem.startShooter();

                }
                if (launcher.getVelocity() > 1375){
                    shooterSubsystem.feedBall();
                }
                if (pathTimer.getElapsedTimeSeconds() > 7.5) {
                    shooterSubsystem.stopFeed();
                    intake.setPower(0);
                    setPathState(PathState.TRIANGLE_DRIVE_SHOOT_ENDPOS);
                }
                break;
            case TRIANGLE_DRIVE_SHOOT_ENDPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosEndPos);
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
        leftFeeder = hardwareMap.get(Servo.class, "left_feeder");
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder");
        launcher = hardwareMap.get(DcMotorEx.class, "shooter");
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
        pathTimer.resetTimer();
    }
    public void loop() {

    follower.update();
    statePathUpdate();
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
