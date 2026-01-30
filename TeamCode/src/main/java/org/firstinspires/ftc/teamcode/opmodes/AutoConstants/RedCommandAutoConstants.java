package org.firstinspires.ftc.teamcode.opmodes.AutoConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedCommandAutoConstants {
    public static final Pose START_POSE = new Pose(121.575, 122.773, Math.toRadians(45));
    public static final Pose SHOOT_POSE = new Pose(94,92, Math.toRadians(50));
    public static final Pose GATHER_POSE_1 = new Pose(88.11996478,85.10895883777242, Math.toRadians(0));
    public static final Pose GATHER_POSE_2 = new Pose(144-19.27358573629768,84.63174114021572,Math.toRadians(0));
    public static final Pose GATHER_POSE_3 = new Pose(144-58.43583535108961,59.38498789346242, Math.toRadians(0));
    public static final Pose GATHER_POSE_4 = new Pose(144-14.469733656174345,59.38498789346242, Math.toRadians(0));
    public static final Pose GATHER_POSE_5 = new Pose(144-19.469733656174345,59.38498789346242, Math.toRadians(0));
    public static final Pose END_POSE = new Pose(144-40.18644067796609,84.87167070217917, Math.toRadians(143.5));
    public PathChain DRIVETOSHOOT;
    public PathChain INTAKELINE1;
    public PathChain LINE1TOSHOOT;
    public PathChain INTAKELINE2;
    public PathChain LINE2TOSHOOT;
    public PathChain INTAKELINE3;
    public PathChain LINE3TOSHOOT;

    public RedCommandAutoConstants(Follower follower) {
        DRIVETOSHOOT = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                START_POSE,

                                SHOOT_POSE
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))

                .build();

        INTAKELINE1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                new Pose(102.226, 82.677),
                                new Pose(136.77, 87.448)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        LINE1TOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(136.77, 84.448),

                                SHOOT_POSE
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        INTAKELINE2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                new Pose(88.354, 56.283),
                                new Pose(136.916, 60.620)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        LINE2TOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.916, 56.620),

                                SHOOT_POSE
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                .build();

        INTAKELINE3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                new Pose(75.933, 29.723),
                                new Pose(132.857, 35.579)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        LINE3TOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(132.857, 35.579),

                                SHOOT_POSE
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}

