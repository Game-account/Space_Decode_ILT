package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;

import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp
@Disabled
public class Teleop extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    
    private PIDController controller;
    public static double p = 0.004, i = 0, d = 0.00003;
    public static double f = 0.005;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor Fly = null;
    private DcMotor Intake = null;
    private Servo Indexer = null;
    private DcMotor Turret = null;
    private Servo upperblocker = null;
    private Servo lowerblocker = null;
    private IMU imu = null;
    private Limelight3A limelight;
    private double lowerblockerhome = 0;
    private double upperblockerhome = 0;
    private double Indexerhome = 0;
    private int[] stored = {0,0,0}; //0 is empty, 1 is purple, 2 is green


    
    private double robotHeading = 0;
    private double turretheading = 0;
    private final double ticks_per_degree_turret = 5.0; // Tune this with TurretCalibration
    private boolean aimbotEnabled = true;

    @Override

    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        Fly = hardwareMap.dcMotor.get("fly");
        Intake = hardwareMap.dcMotor.get("in");
        Indexer = hardwareMap.servo.get("Indexer");
        Turret = hardwareMap.dcMotor.get("Turret");
        upperblocker = hardwareMap.servo.get("upblocker");
        lowerblocker = hardwareMap.servo.get("lowblocker");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); 
        limelight.start();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Fly.setDirection(DcMotorSimple.Direction.FORWARD);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setTargetPosition(0);
        Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Turret.setPower(1.0);

        Indexer.setPosition(Indexerhome);
        upperblocker.setPosition(upperblockerhome);
        lowerblocker.setPosition(lowerblockerhome);


        controller = new PIDController(p, i, d);

        telemetry.addData(">","ready");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()) {
            // Drive logic
            double horizontal = -1.0 * gamepad1.left_stick_x * 0.6;
            double vertical = gamepad1.left_stick_y * 0.6;
            double turn = -1.0 * gamepad1.right_stick_x * 0.6;

            double flPower = vertical + turn + horizontal;
            double frPower = vertical - turn - horizontal;
            double blPower = vertical + turn - horizontal;
            double brPower = vertical - turn + horizontal;
            
            double scaling = Math.max(1.0, Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower))));
            setDrivePower(flPower/scaling, frPower/scaling, blPower/scaling, brPower/scaling);

            // Intake and Flywheel
            if (gamepad1.left_trigger >= 0.05) {
                Intake.setPower(gamepad1.left_trigger);
            } else {
                Intake.setPower(0);
            }
            if (gamepad1.right_trigger >= 0.05) {
                Fly.setPower(gamepad1.right_trigger);
            } else {
                Fly.setPower(0);
            }



            // Turret and Aimbot logic
            aimbotEnabled = (gamepad1.circle || gamepad1.b);
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            
            double relativeTargetAngle;
            if (aimbotEnabled) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double currentTurretAngle = Turret.getCurrentPosition() / ticks_per_degree_turret;
                    relativeTargetAngle = currentTurretAngle + result.getTx();
                } else {
                    relativeTargetAngle = turretheading - robotHeading;
                }
            } else {
                if (gamepad1.left_bumper) { // Using LB to shift manual field target
                    turretheading -= 1;
                } else if (gamepad1.right_stick_button) { // Using stick button to shift manual field target
                    turretheading += 1;
                }
                relativeTargetAngle = turretheading - robotHeading;
            }

            // ±90 degree limits
            if (relativeTargetAngle > 90) relativeTargetAngle = 90;
            if (relativeTargetAngle < -90) relativeTargetAngle = -90;
            
            int turretTargetTicks = (int) (relativeTargetAngle * ticks_per_degree_turret);
            Turret.setTargetPosition(turretTargetTicks);

            telemetry.addData("Heading", robotHeading);
            telemetry.addData("Aimbot", aimbotEnabled);
            telemetry.addData("Turret Target Angle", relativeTargetAngle);
            telemetry.update();
        }
        limelight.stop();
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
