package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.opmode.teleop.Flywheel;


public abstract class AutoHardware extends LinearOpMode {

    HardwareMap hwMap = null;

    // motor configurations
    protected DcMotor backleftDrive = null;

    protected DcMotor backrightDrive = null;
    protected DcMotor frontleftDrive = null;

    protected DcMotor frontrightDrive = null;


    protected ElapsedTime runtime = new ElapsedTime();

    ElapsedTime hogbackSpeedChangeTimer = new ElapsedTime();

    static final double INTAKE_POWER_INTAKE = -0.8;
    static final double INTAKE_POWER_PUSHOUT = 0.8;
    static final double PUSHER_POWER_INTAKE = -0.8;
    static final double PUSHER_POWER_PUSHOUT = 0.8;
    final double HOGBACK_TARGET_INIT_RPM = Flywheel.Config.AUTONOMOUS_DEFAULT_RPM;    // RPM: Rotations Per Minute

    static final double TRIGGER_SHOOT_TIME = 0.4;
    static final double TRIGGER_READY_TIME = 5;
    static final double LAST_TRIGGER_READY_TIME = 2;

    private CRServo pusher0;
    private CRServo pusher1;


    final double STOP_SPEED = 0.0;
    static final double TICKS_PER_REVOLUTION = Flywheel.Config.ENCODER_TICKS_PER_FLYWHEEL_REV;

    final double HOGBACK_TARGET_RANGE = Flywheel.Config.READY_TOLERANCE_RPM;
    static final double MAX_TICKS_PER_SEC = Flywheel.Config.MAX_RPM * TICKS_PER_REVOLUTION / 60.0;

    private double hogback_target_rpm = HOGBACK_TARGET_INIT_RPM;
    private double hogback_target_ticks = hogback_target_rpm * TICKS_PER_REVOLUTION / 60;
    private double hogback_target_ticks_low = (hogback_target_rpm - HOGBACK_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60;
    private ElapsedTime revtime = new ElapsedTime();
    private double minrevtime = Flywheel.Config.AUTONOMOUS_SPINUP_TIMEOUT_SECONDS * 1000.0;


    // For motot encoders
    protected static final double TICKS_DRIVE_PER_REVOLUTION = 537.7;    // GoBilda 312 rpm Yellow Jacket Motor
    protected static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    protected static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    protected static final double TICKS_PER_INCH = (TICKS_DRIVE_PER_REVOLUTION * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // IMU control
    public IMU imu;

    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    private Servo trigger;
    private DcMotorEx hogback;
    private Flywheel flywheel;
    private Limelight3A limelight;

    private DcMotor intake;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;


    static final double TRIGGER_READY = 0.15;
    static final double TRIGGER_SHOOT = 0.55;
    private LAUNCH_STATES launchState;
    private boolean bShootRequested = false;
    private int countShots = 0;

    ElapsedTime triggerShootTimer = new ElapsedTime();
    ElapsedTime triggerReadyTimer = new ElapsedTime();

    public enum LAUNCH_STATES {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,  // trigger SHOOT state
        LAUNCHED,   // trigger back to READY
    }
    // 🔹 UPDATED STATE MACHINE

    public double ticksPerInch = 31.3;
    public double ticksPerDegree = 12;

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
    /**
     * The variable to store our instance of the vision portal.
     */

    // servo to place pixel on backboard
    public void initAll() {
        initServo();
        initMotor();
        initIMU();
    }

    public void initServo() {


    }


    public void initMotor() {
        // Initialize the drive system variables.
        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");
        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");
        // intake motor
        intake = hardwareMap.get(DcMotor.class, "in");
        // trigger servo
        trigger = hardwareMap.get(Servo.class, "arm2");
        // hogback motor with DocMotorEx so that PIDF can be used.
        initFlywheel();

        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);

        hogbackSpeedChangeTimer.reset();
        triggerShootTimer.reset();
        triggerReadyTimer.reset();
        trigger.setPosition(TRIGGER_READY);
        launchState = LAUNCH_STATES.IDLE;

        telemetry.addData("Status", "Initialized");

        // LL camera initialization
        /*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
         */

        telemetry.setMsTransmissionInterval(11);

        // aprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

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
            newFrontLeftTarget = frontleftDrive.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newBackLeftTarget = backleftDrive.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newFrontRightTarget = frontrightDrive.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
            newBackRightTarget = backrightDrive.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);
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
                telemetry.addData("Running to", " %7d :%7d", newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
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
        }
    }

    public void setAutoDriveMotorMode() {
        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initAprilTagDetector() {

    }

    public void stopRobot() {
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
    }

    public void initIMU() {
        // Initialize IMU in the control hub
        //                                                                              Yaw: goes counter-clockwise
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        telemetry.addLine(String.format("INIT Yaw: %.1f\n", getCurrentYaw()));
        telemetry.update();

        // Retrieve the very initial Rotational Angles and Velocities
        imu.resetYaw();
        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = getCurrentYaw();
    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void turnToTargetYaw(double targetYawDegree, double power, long maxAllowedTimeInMills) {
        long timeBegin, timeCurrent;
        double currentYaw = getCurrentYaw();
        ;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nYaw0=%.2f",
                currentYaw, targetYawDegree, yaw0));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 2
                && opModeIsActive()
                && ((timeCurrent - timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 100)
                ticks = 100;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 5)
                break;
            if (diffYaw > 5)
                factor = 1.0;
            else
                factor = diffYaw / 5;
            // factor = 1.0;
            driveMotors(
                    (int) (tickDirection * ticks),
                    (int) (tickDirection * ticks),
                    -(int) (tickDirection * ticks),
                    -(int) (tickDirection * ticks),
                    power * factor, false, 0);
            sleep(30);
            currentYaw = getCurrentYaw();
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double) (timeCurrent - timeBegin)));
            telemetry.update();

        }
    }

    public void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget,
                            double power,
                            boolean bKeepYaw, double targetYaw) {
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
        if (!((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)))
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while (opModeIsActive() &&
                (frontleftDrive.isBusy() &&
                        backleftDrive.isBusy() &&
                        frontrightDrive.isBusy() &&
                        backrightDrive.isBusy())) {
            if (bKeepYaw) {

                currentYaw = getCurrentYaw();
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                } else {
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

    public void turnOnIntakeSubsystem() {
        intake.setPower(INTAKE_POWER_INTAKE);
    }
    public void turnOffIntakeSubsystem() {
        intake.setPower(0);
    }


    // Pushers (CRServo) direct setters (public for fine control)
    public void pusher0Set(double power) {
        if (pusher0 != null) pusher0.setPower(power);
    }

    public void pusher1Set(double power) {
        if (pusher1 != null) pusher1.setPower(power);
    }

    // Trigger controls
    public void triggerReady() {
        if (trigger != null) trigger.setPosition(TRIGGER_READY);
    }

    void launch(int numShots) {
        switch (launchState) {
            case IDLE:
                if (bShootRequested) {
                    revtime.reset();
                    launchState = LAUNCH_STATES.SPIN_UP;

                }
                break;
            case SPIN_UP:
                if (bShootRequested) {
                    if (hogback.getVelocity() > hogback_target_ticks_low || revtime.milliseconds() > minrevtime) {
                        launchState = LAUNCH_STATES.LAUNCH;

                    }
                }
                break;
            case LAUNCH:
                launchState = LAUNCH_STATES.LAUNCHING;
                trigger.setPosition(TRIGGER_SHOOT);
                triggerShootTimer.reset();

                break;
            case LAUNCHING:
                if (triggerShootTimer.seconds() > TRIGGER_SHOOT_TIME) {
                    launchState = LAUNCH_STATES.LAUNCHED;
                    trigger.setPosition(TRIGGER_READY);
                    triggerReadyTimer.reset();

                }
                break;
            case LAUNCHED:
                if (triggerReadyTimer.seconds() > TRIGGER_READY_TIME
                        || ((countShots == (numShots - 1))
                            && triggerReadyTimer.seconds() > LAST_TRIGGER_READY_TIME)) {
                    countShots ++;
                    triggerReadyTimer.reset();
                    if (countShots >= numShots) {
                        // Finished all shots. Stop shooting...
                        bShootRequested = false;
                        launchState = LAUNCH_STATES.IDLE;
                    } else {
                        launchState = LAUNCH_STATES.LAUNCH;
                    }
                }
                break;
        }
        if (bShootRequested) {
            startFlywheel();
        }
        else {
            launchState = LAUNCH_STATES.IDLE;
            stopFlywheel();
        }
        telemetry.addData("State", launchState);
        telemetry.addData("Hogback Velocity Target (RPM)", hogback_target_rpm);
        telemetry.addData("Hogback Velocity Actual (RPM)",
                hogback.getVelocity() / TICKS_PER_REVOLUTION * 60);
        telemetry.addData("Shot", countShots + " of " + numShots);
        telemetry.addData("TriggerShootTimer", triggerShootTimer.seconds());
        telemetry.addData("TriggerReadyTimer", triggerReadyTimer.seconds());
        telemetry.update();
    }

    public void triggerShoot(int numShots) {
        bShootRequested = true;
        countShots = 0;
        try {
            while (opModeIsActive()) {
                launch(numShots);
                if (!bShootRequested) {
                    break;
                }
                idle();
            }
        } finally {
            stopFlywheel();
        }
    }

    public void setTriggerPosition(double position) {
        if (trigger != null) trigger.setPosition(position);
    }

    // Autonomous flywheel helpers. These do not operate the intake, transfer, or trigger.

    /** Initialize only the flywheel, or use initAll() to initialize the existing full robot. */
    public void initFlywheel() {
        if (flywheel == null) {
            hogback = hardwareMap.get(DcMotorEx.class, "fly");
            flywheel = new Flywheel(hogback, hardwareMap);
            setFlywheelTargetRpm(HOGBACK_TARGET_INIT_RPM);
        }
    }

    /** Set a bounded RPM without starting an idle wheel. Safe to call before waitForStart(). */
    public void setFlywheelTargetRpm(double rpm) {
        Flywheel controller = requireFlywheel();
        controller.setTargetRpm(rpm);
        hogback_target_rpm = controller.getTargetRpm();
        hogback_target_ticks = hogback_target_rpm * TICKS_PER_REVOLUTION / 60.0;
        hogback_target_ticks_low = Math.max(0.0,
                (hogback_target_rpm - HOGBACK_TARGET_RANGE) * TICKS_PER_REVOLUTION / 60.0);
        if (!opModeIsActive()) {
            controller.stop();
        }
        controller.update();
    }

    /** Start at the selected RPM without waiting for spin-up. Call after waitForStart(). */
    public boolean startFlywheel() {
        Flywheel controller = requireFlywheel();
        if (!opModeIsActive()) {
            stopFlywheel();
            return false;
        }
        controller.start();
        updateFlywheel();
        return controller.isRunning();
    }

    /** Select an RPM and start without blocking; true means a nonzero speed was commanded. */
    public boolean startFlywheel(double rpm) {
        setFlywheelTargetRpm(rpm);
        return startFlywheel();
    }

    /** Refresh measured RPM/readiness and add telemetry; call telemetry.update() to transmit it. */
    public void updateFlywheel() {
        Flywheel controller = requireFlywheel();
        if (!opModeIsActive()) {
            controller.stop();
        }
        controller.update();
        telemetry.addData("Flywheel State", controller.getState());
        telemetry.addData("Flywheel Target (RPM)", "%.0f", controller.getTargetRpm());
        telemetry.addData("Flywheel Actual (RPM)", "%.0f", controller.getActualRpm());
        telemetry.addData("Shooter Battery", controller.hasValidBatteryVoltage()
                ? String.format("%.2f V", controller.getBatteryVoltage()) : "Unavailable (holding F)");
        telemetry.addData("Flywheel F", "%.2f", controller.getAppliedFeedforward());
    }

    public boolean waitForFlywheelReady() {
        return waitForFlywheelReady(Flywheel.Config.AUTONOMOUS_SPINUP_TIMEOUT_SECONDS);
    }

    /**
     * Wait for stable RPM, leaving the wheel running on success.
     * Returns false and stops on timeout, STOP, interruption, or an idle wheel.
     * A zero timeout performs one immediate readiness check without waiting.
     */
    public boolean waitForFlywheelReady(double timeoutSeconds) {
        validateFlywheelDuration(timeoutSeconds);
        Flywheel controller = requireFlywheel();
        boolean ready = false;
        try {
            double started = flywheelTimeSeconds();
            while (opModeIsActive() && controller.isRunning()) {
                updateFlywheel();
                telemetry.update();
                if (!opModeIsActive()) {
                    return false;
                }
                double elapsed = flywheelTimeSeconds() - started;
                if (controller.isReady() && (timeoutSeconds == 0.0 || elapsed <= timeoutSeconds)) {
                    ready = true;
                    return true;
                }
                if (elapsed >= timeoutSeconds) {
                    return false;
                }
                idle();
            }
            return false;
        } finally {
            if (!ready) {
                stopFlywheel();
            }
        }
    }

    /**
     * Run for a duration measured from the start command, including spin-up, then stop.
     * Returns false if STOP/interruption occurs. Does not start or move other mechanisms.
     * To start timing at operating speed, call startFlywheel()/waitForFlywheelReady() first.
     */
    public boolean runFlywheelFor(double rpm, double durationSeconds) {
        validateFlywheelDuration(durationSeconds);
        if (!Double.isFinite(rpm)) {
            throw new IllegalArgumentException("Flywheel RPM must be finite");
        }
        requireFlywheel();
        try {
            if (durationSeconds == 0.0) {
                return opModeIsActive();
            }
            if (!startFlywheel(rpm)) {
                return false;
            }
            double started = flywheelTimeSeconds();
            while (opModeIsActive()) {
                if (flywheelTimeSeconds() - started >= durationSeconds) {
                    return true;
                }
                updateFlywheel();
                telemetry.update();
                idle();
            }
            return false;
        } finally {
            stopFlywheel();
        }
    }

    /** Command zero output, preserve the selected RPM, and cancel a legacy shooting request. */
    public void stopFlywheel() {
        if (flywheel != null) {
            flywheel.stop();
        }
        bShootRequested = false;
        launchState = LAUNCH_STATES.IDLE;
    }

    public boolean isFlywheelReady() {
        return requireFlywheel().isReady();
    }

    public double getFlywheelTargetRpm() {
        return requireFlywheel().getTargetRpm();
    }

    /** Measured RPM from the most recent updateFlywheel(), wait, or timed run. */
    public double getFlywheelActualRpm() {
        return requireFlywheel().getActualRpm();
    }

    protected double flywheelTimeSeconds() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    private Flywheel requireFlywheel() {
        if (flywheel == null) {
            throw new IllegalStateException("Call initFlywheel() or initAll() before flywheel commands");
        }
        return flywheel;
    }

    private static void validateFlywheelDuration(double seconds) {
        if (!Double.isFinite(seconds) || seconds < 0.0) {
            throw new IllegalArgumentException("Flywheel duration must be finite and nonnegative");
        }
    }

    // Existing autonomous routines can keep using the legacy method names.
    public void setHogbackTargetRpm(double rpm) {
        setFlywheelTargetRpm(rpm);
    }

    public void setHogbackTargetRpmClose() {
        setFlywheelTargetRpm(Flywheel.Config.AUTONOMOUS_CLOSE_RPM);
    }

    public double getHogbackTargetRpm() {
        return getFlywheelTargetRpm();
    }



}
       /* private void driveStrafe(int flTarget, int blTarget, int frTarget, int brTarget,
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

*/

    /*   public void turnToTargetYaw2(double targetYawDegree, double power, long maxAllowedTimeInMills){
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
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks*0.5),
                    -(int)(tickDirection * ticks*0.5),
                    power * factor, false, 0);
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }

     */


    /* public void driveToBackBoardByAprilTag(int targetId) {
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

            for (AprilTagDetection detection : currentDetections) {

                idSeen = detection.id;



                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }

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
*/
  /*  public void strafe(double power, int milliseconds) {

        frontleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontleftDrive.setPower(power);
        backleftDrive.setPower(-power);
        frontrightDrive.setPower(-power);
        backrightDrive.setPower(power);
        sleep(milliseconds);
        turnToTargetYaw2(90+yaw0, 0.4, 5000);
        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);

    }
    public void dropPixelOnLine() {
        grabberR.setPosition(0);
        //grabberL.setPosition(0);
        sleep(1000);
        grabberTilt.setPosition(1);
    }
    public void dropPixelOnBoard() {

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

  */

   /* public void turn(double powerLeft, double powerRight, long milliseconds){
        frontleftDrive.setPower(powerLeft);
        frontrightDrive.setPower(powerRight);
        backleftDrive.setPower(powerLeft);
        backrightDrive.setPower(powerRight);
        sleep(milliseconds);
        stopRobot();
    }
    */



