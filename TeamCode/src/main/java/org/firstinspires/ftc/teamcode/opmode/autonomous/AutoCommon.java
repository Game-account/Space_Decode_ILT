package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

public abstract class AutoCommon extends LinearOpMode {

    // motor configurations
    protected DcMotor         backleftDrive   = null;

    protected DcMotor         backrightDrive  = null;
    protected DcMotor         frontleftDrive   = null;

    protected DcMotor         frontrightDrive  = null;

    //protected DcMotor         rotator  = null;
    //linear slider
    //motors
    protected DcMotor Fly = null;
    protected DcMotor Intake = null;
    protected Servo Feeder = null;
    protected Servo Indexer = null;
    
    
    protected ElapsedTime     runtime = new ElapsedTime();

    // For motot encoders
    protected static final double     COUNTS_PER_MOTOR_REV    = 500 ;    // eg: TETRIX Motor Encoder
    protected static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    protected static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    protected static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    protected static final int RORATE_ARM_TICKS = 1500;
    protected static final double TILT_INIT_POSITION = 0.75;
    // IMU control
    public IMU imu;

    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    public double ticksPerInch = 31.3;
    public double ticksPerDegree = 15;
    // Tensor flow/april tag instance variables
    protected static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    //private static final String TFOD_MODEL_ASSET = "model_20230924.tflite";
    protected static final String TFOD_MODEL_ASSET = "model_cylinder2_20231125.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    protected static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
 /*
    private static final String[] LABELS = {
       "pixel yellow", "pixel white", "pixel green", "pixel purple", "Pixel"
    };
*/
    protected static final String[] LABELS = {
            "cylinder"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //protected TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    protected VisionPortal visionPortal;

    protected AprilTagProcessor aprilTag;
    
    protected double feedermin = 0;
    protected double feedermax = 1;
    protected double Indexermin = 0;
    protected double Indexermax = 1;
    protected double Indexerhome = 0;
    protected double Indexamount = (Indexermax - Indexermin)/6;
    protected double Index = Indexerhome;

    // servo to place pixel on backboard

    public void initAll(){
        //initTfodAndAprilTag();
        initServo();
        initMotor();
        initIMU();
        //initSensors();
    }
    public void initServo() {
        Feeder = hardwareMap.servo.get("Feeder");
        Indexer = hardwareMap.servo.get("Indexer");
        Feeder.setPosition(feedermin);
        Indexer.setPosition(Indexerhome);
    }
    public void initMotor(){
        // Initialize the drive system variables.
        backleftDrive  = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");
        frontleftDrive  = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);

        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initFlyWheel();
    }
    public void initFlyWheel() {
        Fly = hardwareMap.dcMotor.get("Fly");
        Fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake = hardwareMap.dcMotor.get("Intake");
        Fly.setPower(0);
    }

    public void strafe_encoder(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = backleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backrightDrive.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = frontleftDrive.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backleftDrive.setTargetPosition(newBackLeftTarget);
            backrightDrive.setTargetPosition(newBackRightTarget);
            frontleftDrive.setTargetPosition(newFrontLeftTarget);
            frontrightDrive.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backleftDrive.setPower(Math.abs(speed));
            backrightDrive.setPower(Math.abs(speed));
            frontleftDrive.setPower(Math.abs(speed));
            frontrightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backleftDrive.isBusy() && backrightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        backleftDrive.getCurrentPosition(), backrightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = backleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = frontleftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontrightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backleftDrive.setTargetPosition(newBackLeftTarget);
            backrightDrive.setTargetPosition(newBackRightTarget);
            frontleftDrive.setTargetPosition(newFrontLeftTarget);
            frontrightDrive.setTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backleftDrive.setPower(Math.abs(speed));
            backrightDrive.setPower(Math.abs(speed));
            frontleftDrive.setPower(Math.abs(speed));
            frontrightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backleftDrive.isBusy() && backrightDrive.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Running to",  " %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                //telemetry.addData("Currently at",  " at %7d :%7d",backleftDrive.getCurrentPosition(), backrightDrive.getCurrentPosition());
                //telemetry.update();
            }

            // Stop all motion;
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


    public void driveToBackBoardByAprilTag(int targetId) {
        //adjustment position according to apriltag
        double xPos = 10;
        double yPos = 10;
        int maxTries = 0;
        int idSeen = 0;
        int strafetime = 100;
        while (((xPos > 1) || (xPos < -1)) && (maxTries < 100)) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            idSeen = 0;
            // Step through the list of detections and display info for each one.

            for (AprilTagDetection rawDetection : currentDetections) {
                // This routine targets individual tag IDs, not tag clusters.
                if (!(rawDetection instanceof AprilTagSingleDetection)) {
                    continue;
                }
                AprilTagSingleDetection detection = (AprilTagSingleDetection) rawDetection;

                idSeen = detection.id;
/*
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
*/
                if (detection.id == targetId) {
                    xPos = detection.ftcPose.x;
                    break;
                }
                else {
                    if (idSeen > 0) {
                        if (targetId < idSeen) {
                            xPos = -10;
                        } else {
                            xPos = 10;
                        }
                    }
                }

            }   // end for() loop

            if (idSeen > 0) {
                telemetry.addLine(String.format("Target found! xPos is %6.1f (inch)", xPos));
                telemetry.update();
                if (xPos > 6 || xPos < -6) {
                    strafetime = 700;
                }
                else {
                    strafetime = 100;
                }
                if (xPos > -2.0) {
                    strafe(0.3, strafetime);
                } else if (xPos < -4.0) {
                    strafe(-0.3, strafetime);
                } else {
                    break;
                }
            }
            sleep(20);
            maxTries++;
        }
    }

    public void strafe(double power, int milliseconds) {

        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontleftDrive.setPower(power);
        backleftDrive.setPower(-power);
        frontrightDrive.setPower(-power);
        backrightDrive.setPower(power);
        sleep(milliseconds);
        //turnToTargetYaw2(90+yaw0, 0.4, 5000);
        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);

    }

    public void forward(double power, int milliseconds) {
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(power);
        backleftDrive.setPower(power);
        backrightDrive.setPower(power);

        sleep(milliseconds);
        stopRobot();
    }
    public void backward(double power, int milliseconds){
        frontleftDrive.setPower(-power);
        frontrightDrive.setPower(-power);
        backleftDrive.setPower(-power);
        backrightDrive.setPower(-power);

        sleep(milliseconds);
        stopRobot();
    }
    public void stopRobot(){
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
    }

    public void turn(double powerLeft, double powerRight, long milliseconds){
        frontleftDrive.setPower(powerLeft);
        frontrightDrive.setPower(powerRight);
        backleftDrive.setPower(powerLeft);
        backrightDrive.setPower(powerRight);
        sleep(milliseconds);
        stopRobot();
    }

    public void initIMU(){
        // Initialize IMU in the control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        telemetry.addLine(String.format("INIT Yaw: %.1f\n", getCurrentYaw()));
        telemetry.update();

        // Retrieve the very initial Rotational Angles and Velocities
        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);

    }

    public Integer IMUdirection() {
        initIMU();
        return (int) orientation0.getRoll(AngleUnit.DEGREES);
    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Drive to a specific position and angle relative to an AprilTag using Limelight data.
     * @param limelight The Limelight3A object
     * @param targetX Desired X position in robot-centric or target-centric space (e.g. tx error or Pose3D.x)
     * @param targetY Desired Y position
     * @param targetYaw Desired relative angle (using tx or IMU)
     * @param speed Maximum motor power
     * @param timeoutS Maximum time allowed for the move
     * @param direction Multiplier for movement directions (1 or -1)
     */
    public void driveToLimelightPosition(Limelight3A limelight, double targetX, double targetY, double targetYaw, double speed, double timeoutS, int direction) {
        // Proportional constants for P-controller
        double kP_trans = 0.05;
        double kP_turn = 0.02;
        
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            LLResult result = limelight.getLatestResult();
            
            if (result != null && result.isValid()) {
                // For turning, we can use the Limelight's tx directly (angle to target)
                // If we want to face the target, our error is simply result.getTx()
                // If we want to be at a specific targetYaw relative to the target, we adjust.
                double currentTx = result.getTx();
                double turnError = targetYaw - currentTx; // Simplified: aiming to get tx to targetYaw
                
                // For translation (X and Y), we can use result.getTy() for distance 
                // or result.getBotpose_TargetSpace() if AprilTags are configured for 3D.
                // Here we'll use tx and ty proportionally as "X and Y like limelight output"
                double currentTy = result.getTy();
                
                double errorX = targetX - currentTx;
                double errorY = targetY - currentTy;

                // Stop if we are within a reasonable tolerance
                if (Math.abs(errorX) < 1.0 && Math.abs(errorY) < 1.0 && Math.abs(turnError) < 1.0) {
                    break;
                }

                // Calculate basic mecanum powers
                double drive = errorY * kP_trans * direction;
                double strafe = errorX * kP_trans * direction;
                double turn = turnError * kP_turn;

                // Combine powers for mecanum drive
                double fl = drive + strafe + turn;
                double fr = drive - strafe - turn;
                double bl = drive - strafe + turn;
                double br = drive + strafe - turn;

                // Normalize powers to stay within [ -speed, speed ]
                double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
                if (max > speed) {
                    fl = (fl / max) * speed;
                    fr = (fr / max) * speed;
                    bl = (bl / max) * speed;
                    br = (br / max) * speed;
                }

                frontleftDrive.setPower(fl);
                frontrightDrive.setPower(fr);
                backleftDrive.setPower(bl);
                backrightDrive.setPower(br);
                
                telemetry.addData("Limelight Error", "X:%.2f, Y:%.2f, Turn:%.2f", errorX, errorY, turnError);
            } else {
                // If target lost, stop for safety or keep trying until timeout
                stopRobot();
            }
            telemetry.update();
        }
        stopRobot();
    }

    private void driveStrafe(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        int direction;

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setTargetPosition(flTarget);
        backleftDrive.setTargetPosition(blTarget);
        frontrightDrive.setTargetPosition(frTarget);
        backrightDrive.setTargetPosition(brTarget);

        frontleftDrive.setPower(power);
        backleftDrive.setPower(power);
        frontrightDrive.setPower(power);
        backrightDrive.setPower(power);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (frontleftDrive.isBusy() &&
                        backleftDrive.isBusy() &&
                        frontrightDrive.isBusy() &&
                        backrightDrive.isBusy())){
            if (bKeepYaw) {

                currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                frontleftDrive.setPower(powerL);
                backleftDrive.setPower(powerL);
                frontrightDrive.setPower(powerR);
                backrightDrive.setPower(powerR);
            }
            idle();
        }

        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
    }


    public void turnToTargetYaw3(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 130)
                ticks = 130;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;
            factor = 1.0;
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    power * factor, false, 0);
            sleep(30);
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }

    public void turnToTargetYaw2(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 600)
                ticks = 600;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1) {
                break;
            }
            if (diffYaw > 3) {
                factor = 1.0;
            }
            else {
                factor = diffYaw / 3;
            }

                driveMotors(
                        (int) (tickDirection * ticks ),
                        (int) (tickDirection * ticks ),
                        -(int) (tickDirection * ticks),
                        -(int) (tickDirection * ticks),
                        power * factor, false, 0);

            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }
    public void turnToTargetYaw4(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 600)
                ticks = 600;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;

                driveMotors(
                        (int) (tickDirection * ticks),
                        (int) (tickDirection * ticks),
                        -(int) (tickDirection * ticks),
                        -(int) (tickDirection * ticks),
                        power * factor, false, 0);

            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }
    public void turnToTargetYaw(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 200)
                ticks = 200;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    power * factor, false, 0);
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }
    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        double leftRatioToCounterCOG = 0.95;
        int direction;

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setTargetPosition(flTarget);
        backleftDrive.setTargetPosition(blTarget);
        frontrightDrive.setTargetPosition(frTarget);
        backrightDrive.setTargetPosition(brTarget);

        frontleftDrive.setPower(power * leftRatioToCounterCOG);
        backleftDrive.setPower(power * leftRatioToCounterCOG);
        frontrightDrive.setPower(power);
        backrightDrive.setPower(power);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (frontleftDrive.isBusy() &&
                        backleftDrive.isBusy() &&
                        frontrightDrive.isBusy() &&
                        backrightDrive.isBusy())){
            if (bKeepYaw) {

                currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                frontleftDrive.setPower(powerL);
                backleftDrive.setPower(powerL);
                frontrightDrive.setPower(powerR);
                backrightDrive.setPower(powerR);
            }
            idle();
        }

        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
    }
}
