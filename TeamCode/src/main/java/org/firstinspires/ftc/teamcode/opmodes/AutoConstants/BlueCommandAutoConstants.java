package org.firstinspires.ftc.teamcode.opmodes.AutoConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueCommandAutoConstants {
    public static final Pose START_POSE = new Pose(22.425, 122.773, Math.toRadians(135));
    public static final Pose SHOOT_POSE = new Pose(50,92, Math.toRadians(135));
    public static final Pose GATHER_POSE_1 = new Pose(55.88003521901827,85.10895883777242, Math.toRadians(180));
    public static final Pose GATHER_POSE_2 = new Pose(19.27358573629768,84.63174114021572,Math.toRadians(180));
    public static final Pose GATHER_POSE_3 = new Pose(58.43583535108961,59.38498789346242, Math.toRadians(180));
    public static final Pose GATHER_POSE_4 = new Pose(14.469733656174345,59.38498789346242, Math.toRadians(180));
    public static final Pose GATHER_POSE_5 = new Pose(19.469733656174345,59.38498789346242, Math.toRadians(180));
    public static final Pose END_POSE = new Pose(40.18644067796609,84.87167070217917, Math.toRadians(143.5));
    public PathChain DRIVETOSHOOT;
    public PathChain INTAKELINE1;
    public PathChain LINE1TOSHOOT;
    public PathChain INTAKELINE2;
    public PathChain LINE2TOSHOOT;
    public PathChain INTAKELINE3;
    public PathChain LINE3TOSHOOT;

    public BlueCommandAutoConstants(Follower follower) {
        DRIVETOSHOOT = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                START_POSE,

                                SHOOT_POSE
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))

                .build();

        INTAKELINE1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                new Pose(41.774, 82.677),
                                new Pose(17.230, 84.448)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        LINE1TOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.230, 84.448),

                                SHOOT_POSE
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        INTAKELINE2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                new Pose(55.646, 56.283),
                                new Pose(15.084, 59.620)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        LINE2TOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.084, 59.620),

                                SHOOT_POSE
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        INTAKELINE3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                SHOOT_POSE,
                                new Pose(68.067, 29.723),
                                new Pose(14.143, 35.579)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        LINE3TOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.143, 35.579),

                                SHOOT_POSE
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}
