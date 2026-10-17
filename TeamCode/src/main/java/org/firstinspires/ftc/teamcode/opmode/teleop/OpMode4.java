package org.firstinspires.ftc.teamcode.opmode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware4;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OpMode4")
public class OpMode4 extends OpMode {

    RobotHardware4 robot = new RobotHardware4();

    final double FEED_TIME_SECONDS = 0.5;
    final double STOP_SPEED = -300;
    final double FULL_SPEED = -1;

    static final double INTAKE_POWER_INTAKE = -0.8;
    static final double INTAKE_POWER_PUSHOUT = 0.8;
    static final double PUSHER_POWER_INTAKE = -0.8;
    static final double PUSHER_POWER_PUSHOUT = 0.8;
    boolean forward = true;

    final double BUMPER_FEED_TIME = 0.55;
    final double BUMPER_FEED_POWER = -0.3;

    // Constants for GoBilda 5203 6000 rpm motor
    static final double TICKS_PER_REVOLUTION = 28.0;
    static final double MAX_TICKS_PER_SEC = 2800.0;

    final double HOGBACK_TARGET_INIT_RPM = 5100;    // RPM: Rotations Per Minute
    final double HOGBACK_TARGET_RANGE = 80;

    private double hogback_target_rpm = HOGBACK_TARGET_INIT_RPM;
    private double hogback_target_ticks = hogback_target_rpm * TICKS_PER_REVOLUTION / 60;
    private double hogback_target_ticks_low= (hogback_target_rpm - HOGBACK_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60;


    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake, transport;

    private Servo trigger;
    static final double TRIGGER_READY = 0.15;
    static final double TRIGGER_SHOOT = 0.55;
    private DcMotorEx hogback;

    private IMU imu;

    final double transport_power=0.7;


    ElapsedTime hogbackSpeedChangeTimer = new ElapsedTime();
    static final double SPEED_CHANGE_TIME = 0.2; // seconds

    boolean spin;

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
    }
    // 🔹 UPDATED STATE MACHINE

    private LaunchState launchState;
    private boolean ShootRequested = false;

    @Override
    public void init() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "in");
        transport = hardwareMap.get(DcMotor.class, "trans");
        hogback = hardwareMap.get(DcMotorEx.class, "fly");

        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(BRAKE);

        // Set hogback motor to run with ENCODER and set PID coefficients
        hogback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hogback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hogback.setDirection(DcMotor.Direction.REVERSE);
        hogback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hogback.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(350, 0.1, 0, 12)
        );
        hogbackSpeedChangeTimer.reset();

        launchState = LaunchState.IDLE;
        trigger.setPosition(TRIGGER_READY);

        // leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        //   leftFeeder.setPower(STOP_SPEED);
        //   rightFeeder.setPower(STOP_SPEED);


        telemetry.addData("Status", "Initialized");

/*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
 */

        telemetry.setMsTransmissionInterval(11);
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust these two settings to match how your Hub is mounted!
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCameraResolution(new Size(640, 480));
//        builder.addProcessor(aprilTagProcessor);
//        visionPortal = builder.build();
    }


    @Override
    public void loop() {

        double horizontal = -1.0 * gamepad1.left_stick_x * 0.6;
        double vertical = gamepad1.left_stick_y * 0.6;
        double turn = -1.0 * gamepad1.right_stick_x * 0.6;

        //double tx = LimelightHelpers.getTX("limelight");
        // boolean hasTarget = LimelightHelpers.hasTarget("limelight");

        double flPower = vertical + turn + horizontal;
        double frPower = vertical - turn - horizontal;
        double blPower = vertical + turn - horizontal;
        double brPower = vertical - turn + horizontal;
        double scaling = Math.max(1.0,
                Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                        Math.max(Math.abs(blPower), Math.abs(brPower))));
        flPower = flPower / scaling;
        frPower = frPower / scaling;
        blPower = blPower / scaling;
        brPower = brPower / scaling;

        if (leftFrontDrive == null || rightFrontDrive == null || leftBackDrive == null || rightBackDrive == null) {
            telemetry.addData("DriveMotorNull", String.format("lf:%b rf:%b lb:%b rb:%b",
                    leftFrontDrive == null, rightFrontDrive == null, leftBackDrive == null, rightBackDrive == null));
            telemetry.update();
        } else {
            leftFrontDrive.setPower(flPower);
            rightFrontDrive.setPower(frPower);
            leftBackDrive.setPower(blPower);
            rightBackDrive.setPower(brPower);
        }

        if (gamepad1.right_trigger>0.05) {
            spin=true;
        }else if (gamepad1.right_bumper) {
            spin=false;
        }
        // TODOs
        if (gamepad1.y) {
            // telemetry.addLine("x is pressed");
            if (hogbackSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                hogbackSpeedChangeTimer.reset();
                hogback_target_rpm += 100;
            }
        }
        else if (gamepad1.a){
            // telemetry.addLine("b is pressed");
            if (hogbackSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                hogbackSpeedChangeTimer.reset();
                hogback_target_rpm -= 100;
            }
        }else if(gamepad1.b){
            if (hogbackSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                hogbackSpeedChangeTimer.reset();
                hogback_target_rpm = 5350;
            }
        }else if (gamepad1.x){
            if (hogbackSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                hogbackSpeedChangeTimer.reset();
                hogback_target_rpm = 5100;
            }
        }
        hogback_target_ticks = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                hogback_target_rpm * TICKS_PER_REVOLUTION / 60));
        hogback_target_ticks_low = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                (hogback_target_rpm - HOGBACK_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60));

        launch(spin);

// intake & transfer subsystems synchronized action
// Left bumper: start intake motor, pusher0, pusher1
// Right bumper: reverse intake motor, pusher0, pusher1
// Left trigger: stop intake motor, pusher0, pusher1
        if (gamepad1.left_trigger > 0.05) {
            telemetry.addLine("left bumper is pressed. Start intake subsystem");
            intake.setPower(INTAKE_POWER_INTAKE);
        }
//        else if (gamepad1.right_bumper) {
//            telemetry.addLine("right bumper is pressed. Reverse intake subsystem");
//            intake.setPower(INTAKE_POWER_PUSHOUT);
//        }

        else if (gamepad1.left_bumper) {
            telemetry.addLine("stop intake subsystem");
            intake.setPower(0);
        }


//hogback launch state
        telemetry.addData("State", launchState);
        telemetry.addData("Hogback Velocity Target (RPM)", hogback_target_rpm);
        telemetry.addData("Hogback Velocity Actual (RPM)",
                hogback.getVelocity() / TICKS_PER_REVOLUTION * 60);
    }


    void launch(boolean spin) {
        switch (launchState) {
            case IDLE:
                if (spin) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (ShootRequested) {
                    if (hogback.getVelocity() > hogback_target_ticks_low) {
                        launchState = LaunchState.LAUNCH;
                    }
                }
                if(!spin) {
                    launchState = LaunchState.IDLE;
                    hogback.setVelocity(STOP_SPEED);
                }
                break;
            case LAUNCH:
                if (!spin) {
                    launchState = LaunchState.IDLE;
                    hogback.setVelocity(STOP_SPEED);
                }
                // trigger to shoot.
                //  Automatically make trigger ready after TRIGGER_SHOOT_TIME
                if (gamepad1.dpad_up && hogback.getVelocity() > hogback_target_ticks_low){
                    telemetry.addLine("dpad up is pressed");
                    transport.setPower(transport_power);
                }
                if (gamepad1.dpad_down){
                    telemetry.addLine("dpad down");
                    transport.setPower(0);
                }
                break;
        }
        if (ShootRequested) {
            hogback.setVelocity(hogback_target_ticks);
        }
    }
}