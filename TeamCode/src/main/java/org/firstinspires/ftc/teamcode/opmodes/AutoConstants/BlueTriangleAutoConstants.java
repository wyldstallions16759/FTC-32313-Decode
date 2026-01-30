package org.firstinspires.ftc.teamcode.opmodes.AutoConstants;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BlueTriangleAutoConstants {
    public PathChain TURNTOSHOOT;
    public PathChain INTAKELINE1;
    public PathChain LINE1TOSHOOT;
    public PathChain LEAVE;

    public BlueTriangleAutoConstants(Follower follower) {
        TURNTOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(56.000, 8.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(113))
                .build();

        INTAKELINE1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(78.878, 39.585),
                                new Pose(12.390, 36.927)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        LINE1TOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.390, 36.927),

                                new Pose(56.000, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113))

                .build();

        LEAVE = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(37.390, 23.244)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}