package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@TeleOp(name = "32313 TeleOp", group = "StarterBot")
//@Disabled
public class Team_TeleOp extends OpMode {
    final double FEED_TIME_SECONDS = 0.20;
    final double STOP_SPEED = 0.5;
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_TARGET_VELOCITY = 2125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private Servo leftFeeder = null;
    private Servo rightFeeder = null;
    private DcMotor intakeMotor = null;
    private GoBildaPinpointDriver pinpoint = null;
    private ShooterSubsystem shooterSubsystem;
    private double axial;
    private double lateral;
    private double yaw;
    private double  P = 450;
    private double F = 1.49;
    private double D = 0;
    private double LowVelocity = 900;
    private double HighVelocity = 1600;
    private double currentVelocity = HighVelocity;
    double[] StepSizes = {10, 1, 0.1, 0.01, 0};
    int StepIndex = 1;
    boolean SlowOn = false;
    private boolean stepperkeydown = false;
    private int launchVelo = 1550;
    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    boolean lastAState2 = false;
    boolean lastAState  = false;
    boolean intakeOn = false;
    boolean lastYstate = false;
    boolean launcherOn = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "shooter"); // extension hub 1
        leftFeeder = hardwareMap.get(Servo.class, "left_feeder"); //extension hub 0
        rightFeeder = hardwareMap.get(Servo.class, "right_feeder"); //extension hub 1
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor"); //control hub 3


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* ZA WARUDO
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 8.2, 11.49));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        axial = -gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;

        // Pressing the Y button makes the Launcher turn on
        // Pressing the Y button again makes the Launcher turn off
        // lastYstate is whether or not the launcher is on

        boolean stepup = gamepad2.right_bumper;
        boolean stepdown = gamepad2.left_bumper;


        if (stepup && !stepperkeydown){
            this.launchVelo=1800;
        }
        else if (stepdown && !stepperkeydown){
            this.launchVelo=1550;
        }

        if (gamepad2.right_trigger > 0.5) {
            launcher.setVelocity(launchVelo);
        } else {
            launcher.setVelocity(STOP_SPEED);
        }
            if (gamepad2.left_trigger > 0.02) {
                intakeMotor.setPower(-gamepad2.left_trigger);
            } else if (gamepad2.x) {
                intakeMotor.setPower(1);
            } else if (gamepad1.right_trigger > 0.02) {
                intakeMotor.setPower(-gamepad1.right_trigger);
            } else {
                intakeMotor.setPower(0);
            }
            if (gamepad2.a) {

                rightFeeder.setPosition(0);
                leftFeeder.setPosition(1);
            } else if (gamepad2.x) {
                rightFeeder.setPosition(1);
                leftFeeder.setPosition(0);
            } else {
                rightFeeder.setPosition(0.5);
                leftFeeder.setPosition(0.5);
            }
            mecanumDrive();
            telemetry.addData("State", launchState);
            telemetry.addData("motorSpeed", launcher.getVelocity());
            telemetry.addData("P", P);
            telemetry.addData("F", F);
            telemetry.addData("D:", D);
            telemetry.addData("A is pressed", gamepad2.a);
            telemetry.addData("Last a state", lastAState2);
            telemetry.addData("Step Index:", StepIndex);
            telemetry.addData("StepSizes:", StepSizes);
            telemetry.addData("Launch Velo", launchVelo);


            lastAState2 = gamepad2.a;
            lastYstate = gamepad2.right_bumper;
            stepperkeydown = stepdown || stepup;
            telemetry.update();
        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        mecanumDrive();
    }
    void toggleShooter(){
        if (currentVelocity == HighVelocity) {
            currentVelocity = LowVelocity;
        }
        else {
            currentVelocity = HighVelocity;
        }
    }
    void mecanumDrive(){

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
    }
    void mecanumSpeedToggle(){
        if (axial == (0.5* -gamepad1.left_stick_y)) {
            SlowOn = false;
            axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;
        }
        else {
            SlowOn = true;
            axial   = 0.5 * -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            lateral =  0.5 * gamepad1.left_stick_x;
            yaw     =  0.5 * gamepad1.right_stick_x;
        }
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(15);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPosition(1);
                rightFeeder.setPosition(0);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPosition(STOP_SPEED);
                    rightFeeder.setPosition(STOP_SPEED);
                }
                break;
        }
    }
}
