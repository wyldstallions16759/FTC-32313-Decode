/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*⠀https://downloads.khinsider.com/game-soundtracks/album/jojo-s-bizarre-adventure-2nd-season-op/01.%2520BLOODY%2520STREAM.mp3⠀⠀
 *⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣀⣤⣠⡤⢶⣶⠖⢲⣶⠀⠀⠀⠀⠀⠀⠀⠀⣼⠋⠉⠉⢹⣷⠖⣶⠖⣶⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⡤⠶⠚⠋⠉⢹⣿⡏⡀⣼⡇⠀⣿⡏⠀⠀⠀⠀⠀⠀⠀⢰⠇⠀⠀⢠⣿⣿⣴⣿⣤⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢰⠏⠁⠀⠀⠀⠀⢐⣿⣿⣀⣰⣿⣶⣾⣿⣥⠤⠴⠖⠒⠒⠚⣿⡿⠶⠿⠿⣿⣿⠋⠉⠉⣿⣏⣀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢀⡟⠀⠀⠀⣀⣠⣴⣾⣿⠿⠛⠛⠋⢹⡿⠀⠀⠀⠀⠀⠀⠀⢰⣿⠁⠀⠀⢰⣿⠇⠀⠀⢸⡿⠀⠀⠀⠀⠀⠉⢹⣿⠀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣼⣁⣤⣶⣿⠿⠟⢻⣿⡟⠀⠀⠀⢀⣿⣁⣀⣤⣴⡶⠀⠀⠀⣾⣇⣀⣀⣤⣾⡟⠀⠀⢀⣿⠷⠶⠶⠶⠆⠀⠀⣾⡿⠋⠁⠀⢀⠉⠑⢦⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⢠⡿⠟⠋⠁⠀⠀⠀⣾⣿⠁⠀⠀⠀⣼⠟⠛⠉⠉⠀⠀⠀⠀⣸⣿⠿⠿⠿⠛⠋⠀⠀⠀⣸⣯⣤⣤⣀⡀⠀⠀⢰⡏⠀⣼⠏⠀⣾⣧⠀⠈⣇⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⡟⠀⠀⠀⠀⠀⠀⢰⣿⠇⠀⠀⠀⢰⡟⠀⣀⣀⣤⠀⠀⠀⢀⡿⠁⠀⠀⠀⠀⠀⠀⠀⣰⡿⠁⠀⠈⠉⠀⠀⢀⣿⡄⠀⠋⢀⣾⠟⠁⠀⣰⡟⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⣸⠁⠀⠀⠀⣀⣤⣶⣿⡏⠀⠀⠀⠀⣾⠿⠛⠛⠉⠁⠀⠀⢀⣾⠇⠀⢀⣀⣀⣀⣀⣤⣾⣿⣇⣀⣀⣀⣠⣤⣴⡿⣏⣻⣦⣴⣿⣷⣀⣠⣴⣿⣁⣀⣠⣤⣀⠀⠀⠀
⠀⠀⠀⠀⢠⠇⣀⣴⣶⣿⣿⠿⠛⠉⠀⠀⠀⠀⣼⠇⠀⠀⠀⠀⢀⣠⣴⣿⣿⣩⠭⠭⠖⠲⣬⣭⢡⣶⢰⢿⣧⣶⣰⢿⡏⠁⢠⡟⡽⣭⣭⣭⣭⡽⢭⣽⢿⣿⣍⣵⣶⣜⢷⣄⠀
⠀⠀⠀⠀⠾⢿⣿⠿⠛⠉⠀⠀⠀⠀⠀⠀⢀⣼⡿⢀⣠⣤⣶⡾⠿⠛⠉⣿⡕⣿⣣⡴⠲⣼⣿⣵⣟⣳⣶⣿⣿⣿⡟⣾⣥⣤⣿⣸⠁⣿⣷⣶⡗⢠⣿⣿⣿⣯⣋⠾⣽⣾⣗⣽⠇
⠀⠀⠀⣴⠞⠉⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⡿⠛⠻⠟⠛⠉⠀⠀⠀⠀⣼⠇⣿⣿⣿⣿⡟⢻⢿⣿⣿⣿⣿⣿⡿⣿⣴⣷⣌⣛⣣⠿⢿⣿⣿⡿⢿⣿⣟⣽⣿⣷⡿⣞⣿⣇⡾⠁⠀
⠀⠀⢠⠇⠀⠀⠀⠀⠀⠀⠀⣠⣴⣿⠟⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣿⣸⢿⣻⣹⣿⢃⡏⢸⠛⡿⣻⣭⢹⣿⢋⣿⣋⣿⣟⡟⢠⣿⣿⣿⠃⡾⡿⣿⣿⣿⣿⢣⣿⡟⣼⠃⠀⠀
⠀⠀⡟⠀⠀⠀⠀⠀⣀⣴⣾⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡾⣣⡿⠿⣿⣻⡏⣾⣟⣩⣴⣇⡽⣷⠿⣷⣿⣾⣿⡞⡿⠁⣾⣿⣿⠏⣸⣽⣧⣿⡿⣞⡿⣎⢣⣻⡇⠀⠀⠀
⠀⣸⠀⠀⠀⣀⣴⣾⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣟⣽⠷⣖⣒⣒⡲⢼⣛⣯⣷⣘⣿⣯⣵⠧⠽⣦⣭⣿⣙⣓⣚⣿⣿⣓⣒⣋⣛⣃⣘⣻⡽⢾⣝⠾⣽⠃⠀⠀⠀
⢠⠇⣀⣴⣾⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⣯⡼⠞⠛⠉⠁⠉⠙⠷⠟⠀⠘⣿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠀⠀⠀⠙⠿⠏⠀⠀⠀⠀*/

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

/*
 *
 * Nicu Nicu Very Nice Ceaser-Chan
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PDiddy" coefficients with some that are a better fit for our application.
 *
 */

@TeleOp(name = "32313 TeleOp", group = "StarterBot")
//@Disabled
public class Team_TeleOp extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.5; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     * YES! YES! YES! YES!
     */
    final double LAUNCHER_TARGET_VELOCITY = 2125; //Green Wheel Speed DO NOT CHANGE IT ISNT THE LAUNCHER
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
    private double  P,F;
    private double LowVelocity = 900;
    private double HighVelocity = 1600;
    private double currentVelocity = HighVelocity;
    double[] StepSizes = {10, 1, 0.1, 0.01, 0};
    int StepIndex = 1;
    boolean SlowOn = false;

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

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

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
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         *
         *  /\_______/\
         * /           \
         * | <O>   <O> |
         * |           |
         * |    ___    |
         * \           /
         *  |         |
         */
        mecanumDrive();

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot. */


        // Pressing the Y button makes the Launcher turn on
        // Pressing the Y button again makes the Launcher turn off
        // lastYstate is whether or not the launcher is on

        if (gamepad2.right_trigger > 0.5) {
            launcher.setVelocity(1650);
        } 
        //else {
        //    launcher.setVelocity(STOP_SPEED);
        //}
        if (gamepad1.a && !lastAState) {
            mecanumSpeedToggle();
        }
        if (gamepad1.xWasPressed()) {
            StepIndex = (StepIndex + 1) % StepSizes.length;
        }
        if (gamepad1.bWasPressed()) {
            StepIndex = (StepIndex - 1) % StepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F += StepSizes[StepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F -= StepSizes[StepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += StepSizes[StepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= StepSizes[StepIndex];
        }
        if (gamepad1.yWasPressed()) {
            toggleShooter();
        }
         if (gamepad2.left_trigger > 0.02) {
             intakeMotor.setPower(-gamepad2.left_trigger);
         }
         else if (gamepad1.right_trigger > 0.02) {
             intakeMotor.setPower(-gamepad1.right_trigger);
         }else if (gamepad2.x){
             intakeMotor.setPower(1);
         }else{
             intakeMotor.setPower(0);
         }
        if (gamepad2.a){

            rightFeeder.setPosition(0);
            leftFeeder.setPosition(1);
        } else if(gamepad2.x){
            rightFeeder.setPosition(1);
            leftFeeder.setPosition(0);
        }else {
            rightFeeder.setPosition(0.5);
            leftFeeder.setPosition(0.5);
        }
        mecanumDrive();

//        if (gamepad2.x) {
//            intake.setPower(1);
//            rightFeeder.setPower(-1);
//            leftFeeder.setPower(-1);
//        }else {
//            intake.setPower(0);
//            rightFeeder.setPower(0);
//            leftFeeder.setPower(0);
//        }
//            intakeOn = !intakeOn;
//        }
//        if (intakeOn) {
//            intake.setPower(-1);
//        } else {
//            intake.setPower(0);
//        }
        // Holding down the dpad_down button makes the feeder turn on
        // Releasing the dpad_down button makes the feeder turn off

        // Holding down the dpad_left button makes the green flywheel go in reverse
        // Releasing the dpad_left button makes the green flywheel stop

//        if (gamepad2.dpad_left) {
//            leftFrontDrive.setPower(1);
//        }
//        if (gamepad2.dpad_up) {
//            rightFrontDrive.setPower(1);
//        }
//        if (gamepad2.dpad_right) {
//            rightBackDrive.setPower(1);
//        }
//        if (gamepad2.dpad_down) {
//            leftBackDrive.setPower(1    );
//        }

        /*
         * Now we call our "Launch" function. 
         */

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("A is pressed", gamepad2.a);
        telemetry.addData("Last a state", lastAState2);
        

        lastAState2 = gamepad2.a;
        lastYstate = gamepad2.right_bumper;
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        mecanumDrive();
    }
    void toggleShooter() {
        if (currentVelocity == HighVelocity) {
            currentVelocity = HighVelocity;
        }
        else {
            currentVelocity = LowVelocity;
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
                launcher.setVelocity(1000);
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
/*⣿⣿⣿⣿⣿⣿⣿⡿⡛⠟⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⣿⣿⠿⠨⡀⠄⠄⡘⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⣿⠿⢁⠼⠊⣱⡃⠄⠈⠹⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⡿⠛⡧⠁⡴⣦⣔⣶⣄⢠⠄⠄⠹⣿⣿⣿⣿⣿⣿⣿⣤⠭⠏⠙⢿⣿⣿⣿⣿⣿
⣿⡧⠠⠠⢠⣾⣾⣟⠝⠉⠉⠻⡒⡂⠄⠙⠻⣿⣿⣿⣿⣿⡪⠘⠄⠉⡄⢹⣿⣿⣿⣿
⣿⠃⠁⢐⣷⠉⠿⠐⠑⠠⠠⠄⣈⣿⣄⣱⣠⢻⣿⣿⣿⣿⣯⠷⠈⠉⢀⣾⣿⣿⣿⣿
⣿⣴⠤⣬⣭⣴⠂⠇⡔⠚⠍⠄⠄⠁⠘⢿⣷⢈⣿⣿⣿⣿⡧⠂⣠⠄⠸⡜⡿⣿⣿⣿
⣿⣇⠄⡙⣿⣷⣭⣷⠃⣠⠄⠄⡄⠄⠄⠄⢻⣿⣿⣿⣿⣿⣧⣁⣿⡄⠼⡿⣦⣬⣰⣿
⣿⣷⣥⣴⣿⣿⣿⣿⠷⠲⠄⢠⠄⡆⠄⠄⠄⡨⢿⣿⣿⣿⣿⣿⣎⠐⠄⠈⣙⣩⣿⣿
⣿⣿⣿⣿⣿⣿⢟⠕⠁⠈⢠⢃⢸⣿⣿⣶⡘⠑⠄⠸⣿⣿⣿⣿⣿⣦⡀⡉⢿⣧⣿⣿
⣿⣿⣿⣿⡿⠋⠄⠄⢀⠄⠐⢩⣿⣿⣿⣿⣦⡀⠄⠄⠉⠿⣿⣿⣿⣿⣿⣷⣨⣿⣿⣿
⣿⣿⣿⡟⠄⠄⠄⠄⠄⠋⢀⣼⣿⣿⣿⣿⣿⣿⣿⣶⣦⣀⢟⣻⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⣿⡆⠆⠄⠠⡀⡀⠄⣽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
⣿⣿⡿⡅⠄⠄⢀⡰⠂⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿*/
/*TO BE CONTINUED*/
