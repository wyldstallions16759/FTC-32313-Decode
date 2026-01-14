package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.sun.tools.javac.code.Lint.LintCategory.PATH;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.5)
            .forwardZeroPowerAcceleration(-24.850539385)
            .lateralZeroPowerAcceleration(-43.240959278)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.75, 0, 0.005, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0275, 0, 0.000005, 0.75,0.01));
            /* .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)

            .centripetalScaling(0.0005);*/
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("right_front_drive")
            .rightRearMotorName("right_back_drive")
            .leftRearMotorName("left_back_drive")
            .leftFrontMotorName("left_front_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(57.397672)
            .yVelocity(48.252816);
            

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.36)
            .strafePodX(-0.23)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
