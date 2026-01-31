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
    final double STOP_SPEED = 0.0;
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
    private DcMotor intake;

    private CRServo pusher0;
    private CRServo pusher1;

    private Servo trigger;
    static final double TRIGGER_READY = 0.15;
    static final double TRIGGER_SHOOT = 0.55;
    private DcMotorEx hogback;
    private Limelight3A limelight;

    private IMU imu;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    ElapsedTime triggerTimer = new ElapsedTime();
    static final double TRIGGER_SHOOT_TIME = 0.4;

    ElapsedTime hogbackSpeedChangeTimer = new ElapsedTime();
    static final double SPEED_CHANGE_TIME = 0.2; // seconds

    int maxShots = 0;

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,  // trigger SHOOT state
        LAUNCHED,   // trigger back to READY
    }
    // 🔹 UPDATED STATE MACHINE

    private LaunchState launchState;
    private boolean bShootRequested = false;

    @Override
    public void init() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "in");
        trigger = hardwareMap.get(Servo.class, "arm2");
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
                new PIDFCoefficients(300, 0, 0, 10)
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

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCameraResolution(new Size(640, 480));
//        builder.addProcessor(aprilTagProcessor);
//        visionPortal = builder.build();
    }


    @Override
    public void loop() {

        double horizontal = -1.0 * gamepad1.right_stick_x * 0.6;
        double vertical = gamepad1.right_stick_y * 0.6;
        double turn = -1.0 * gamepad1.left_stick_x * 0.6;

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



//limelight
        // --- Limelight Logic (Replaces your while loop) ---
        /*
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Botpose", botpose !=null ? botpose.toString(): "null");
        }
        // Sending numbers to Python
        double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
        limelight.updatePythonInputs(inputs);

        // Getting numbers from Python
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            telemetry.addData("Python output:", pythonOutputs[0]);
        }

        // First, tell Limelight which way your robot is facing
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }
*/

//hogback control
        if (gamepad2.y) {
            bShootRequested = true;
        }
        else if (gamepad2.a) {
            bShootRequested = false;
        }
        // TODOs
        else if (gamepad2.b) {
            // telemetry.addLine("x is pressed");
            if (hogbackSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                hogbackSpeedChangeTimer.reset();
                hogback_target_rpm += 100;
            }
            hogback_target_ticks = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                    hogback_target_rpm * TICKS_PER_REVOLUTION / 60));
            hogback_target_ticks_low = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                    (hogback_target_rpm - HOGBACK_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60));
        }
        else if (gamepad2.x){
            // telemetry.addLine("b is pressed");
            if (hogbackSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                hogbackSpeedChangeTimer.reset();
                hogback_target_rpm -= 100;
            }
            hogback_target_ticks = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                    hogback_target_rpm * TICKS_PER_REVOLUTION / 60));
            hogback_target_ticks_low = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                    (hogback_target_rpm - HOGBACK_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60));
        }else if(gamepad2.dpad_right){
            if (hogbackSpeedChangeTimer.seconds() > SPEED_CHANGE_TIME) {
                hogbackSpeedChangeTimer.reset();
                hogback_target_rpm = 5350;
            }
            hogback_target_ticks = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                    hogback_target_rpm * TICKS_PER_REVOLUTION / 60));
            hogback_target_ticks_low = Math.max(0, Math.min(MAX_TICKS_PER_SEC,
                    (hogback_target_rpm - HOGBACK_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60));
        }

        launch(bShootRequested);

// intake & transfer subsystems synchronized action
// Left bumper: start intake motor, pusher0, pusher1
// Right bumper: reverse intake motor, pusher0, pusher1
// Left trigger: stop intake motor, pusher0, pusher1
        if (gamepad2.left_bumper) {
            telemetry.addLine("left bumper is pressed. Start intake subsystem");
            intake.setPower(INTAKE_POWER_INTAKE);
        }
        else if (gamepad2.right_bumper) {
            telemetry.addLine("right bumper is pressed. Reverse intake subsystem");
            intake.setPower(INTAKE_POWER_PUSHOUT);
        }
        else if (gamepad2.left_trigger > 0.5) {
            telemetry.addLine("stop intake subsystem");
            intake.setPower(0);
        }

        // trigger to shoot.
        //  Automatically make trigger ready after TRIGGER_SHOOT_TIME
        if (gamepad2.dpad_up){
            telemetry.addLine("dpad up is pressed");
            trigger.setPosition(TRIGGER_SHOOT);
            triggerTimer.reset();
        }
        if (gamepad2.dpad_down || triggerTimer.seconds() > TRIGGER_SHOOT_TIME){
            telemetry.addLine("dpad down is pressed OR timeout");
            trigger.setPosition(TRIGGER_READY);
        }

//hogback launch state
        telemetry.addData("State", launchState);
        telemetry.addData("Hogback Velocity Target (RPM)", hogback_target_rpm);
        telemetry.addData("Hogback Velocity Actual (RPM)",
                hogback.getVelocity() / TICKS_PER_REVOLUTION * 60);
}


    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayedDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {
            return;
        }

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n====(ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    // A stops hogback


    void launch(boolean shotRequested) {


        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                if (shotRequested) {
                    if (hogback.getVelocity() > hogback_target_ticks_low) {
                        launchState = LaunchState.LAUNCH;
                    }
                }
                else {
                    launchState = LaunchState.IDLE;
                    hogback.setVelocity(STOP_SPEED);
                }
                break;
            case LAUNCH:
                if (!bShootRequested) {
                    launchState = LaunchState.IDLE;
                    hogback.setVelocity(STOP_SPEED);
                }
                break;
       }
        if (bShootRequested) {
            hogback.setVelocity(hogback_target_ticks);
        }
    }



//    public AprilTagDetection getTagBySpecificId(int id) {
//        for (AprilTagDetection detection : detectedTags) {
//            if (detection.id == id) {
//                return detection;
//
//            }
//        }
//        return null;
//
//    }
//
//
//    public void stop() {
//        if (visionPortal != null) {
//            visionPortal.close();
//        }
//    }
}




// Right bumper jogs feeders for 0.1s
//        if (gamepad1.right_bumper && launchState == LaunchState.IDLE) {
//            feederTimer.reset();
//            leftFeeder.setPower(BUMPER_FEED_POWER);
//            rightFeeder.setPower(BUMPER_FEED_POWER);
//            launchState = LaunchState.JOG_FEEDER;
//        }

// launch(maxShots > 0);

//        telemetry.addData("State", launchState);
//        telemetry.addData("Hogback Velocity", hogback.getVelocity());
//

