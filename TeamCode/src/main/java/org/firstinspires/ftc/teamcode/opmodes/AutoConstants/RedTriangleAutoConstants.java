package org.firstinspires.ftc.teamcode.opmodes.AutoConstants;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedTriangleAutoConstants {
    public PathChain INTAKELINE1;
    public PathChain LINE1TOSHOOT;
    public PathChain LEAVE;

    public RedTriangleAutoConstants(Follower follower) {
        INTAKELINE1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 8.000),
                                new Pose(65.122, 39.585),
                                new Pose(131.610, 36.927)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        LINE1TOSHOOT = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(131.610, 36.927),

                                new Pose(88.000, 8.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))

                .build();

        LEAVE = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.000, 8.000),

                                new Pose(106.610, 23.244)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}
