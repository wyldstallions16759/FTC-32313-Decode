package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import com.pedropathing.paths.PathChain;

public class AUTOlogic {

    public Follower follower;
    public Servo leftFeeder;
    public Servo rightFeeder;

    public DcMotorEx launcher = null;
    public DcMotor intake = null;
    public Timer pathTimer = new Timer();
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
    public Pose startPose = new Pose(0, 0, Math.toRadians(0));
    public Pose ShootingPose = new Pose(0,0, Math.toRadians(0));
    public Pose GatherPose1 = new Pose(0,0, Math.toRadians(0));
    public Pose GatherPose2 = new Pose(0,0,Math.toRadians(0));
    public Pose GatherPose3 = new Pose(0,0, Math.toRadians(0));
    public Pose GatherPose4 = new Pose(0,0, Math.toRadians(0));
    public Pose GatherPose5 = new Pose(0,0, Math.toRadians(0));
    public Pose endPose = new Pose(0,0, Math.toRadians(0));

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
    private ShooterSubsystem shooterSubsystem;
    public PathState pathState;
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

                    setPathState(PathState.DRIVE_GATHERPOSE_1_GATHERPOSE_2);
                    pathTimer.resetTimer();
                }

                break;
            case DRIVE_GATHERPOSE_1_GATHERPOSE_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(driveGatherPos1GatherPos2, true);
                    intake.setPower(-1);
                    setPathState(PathState.DRIVE_GATHERPOSE_2_SHOOT);
                    pathTimer.resetTimer();
                }

                break;
            case DRIVE_GATHERPOSE_2_SHOOT:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.followPath(driveGatherPos2ShootPos, true);
                    intake.setPower(-0.5);
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
                    follower.followPath(driveShootPosGatherPos3, true);
                    setPathState(PathState.DRIVE_GATHERPOSE_3_GATHERPOSE_4);
                }
                break;
            case DRIVE_GATHERPOSE_3_GATHERPOSE_4:
                if (!follower.isBusy()) {
                    follower.followPath(driveGatherPos3GatherPos4, true);
                    setPathState(PathState.DRIVE_GATHERPOSE_4_GATHERPOSE_5);
                }
                break;
            case DRIVE_GATHERPOSE_4_GATHERPOSE_5:
                if (!follower.isBusy()) {
                    follower.followPath(driveGatherPos4GatherPos5, true);
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
                follower.followPath(driveStartPosShootPos, true);
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
                    follower.followPath(driveShootPosEndPos, true);
                }

            default:
                break;

        }
    }
}

