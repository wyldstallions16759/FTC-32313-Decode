package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@Autonomous
public class AutoPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;


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


    private final Pose startPose = new Pose(21.487750556792875, 123.47438752783964, Math.toRadians(138));
    private final Pose ShootingPose = new Pose(54.6815144766147,88.35634743875278, Math.toRadians(138));
    private final Pose endPose = new Pose(57.832503374390456,103.61205502050764, Math.toRadians(90));

    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    private ShooterSubsystem shooterSubsystem;
    public void buildPaths() {
        // put in start and end pos
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ShootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ShootingPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(ShootingPose,endPose))
                .setLinearHeadingInterpolation(ShootingPose.getHeading(),endPose.getHeading())
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
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    shooterSubsystem.startShooter();
                    telemetry.addLine("Done Path 1!");
                    follower.followPath(driveStartPosShootPos, true);
                    setPathState(PathState.DRIVE_SHOOT_END);
                }
                break;
            case DRIVE_SHOOT_END:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 7) {
                    telemetry.addLine("Done all paths");

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
    telemetry.addData("Path timer", pathTimer.getElapsedTimeSeconds());
    telemetry.update();
    }
}
