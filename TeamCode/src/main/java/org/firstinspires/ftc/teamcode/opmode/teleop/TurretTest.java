package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Turret and IMU Test", group = "Test")
@Disabled
public class TurretTest extends LinearOpMode {

    private DcMotor turret = null;
    private IMU imu = null;

    // Use the TurretCalibration file to find this exact value.
    // This maps 1 degree of rotation to X encoder ticks.
    private final double ticksPerDegreeTurret = 5.0; 

    private double turretFieldHeading = 0;
    private double robotHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Turret Motor
        turret = hardwareMap.get(DcMotor.class, "Turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setPower(1.0);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        // Orientation: Logo facing UP, USB facing DOWN
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get robot heading
            robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Manual adjustment of turret field heading
            if (gamepad1.dpad_left) {
                turretFieldHeading -= 1.0;
            } else if (gamepad1.dpad_right) {
                turretFieldHeading += 1.0;
            }

            // Reset field heading to 0
            if (gamepad1.y) {
                turretFieldHeading = 0;
            }

            // Calculate relative angle to maintain field heading
            double relativeTargetAngle = turretFieldHeading - robotHeading;

            // Apply ±90 degree limits relative to robot front
            if (relativeTargetAngle > 90) relativeTargetAngle = 90;
            if (relativeTargetAngle < -90) relativeTargetAngle = -90;

            // Convert to ticks and move
            int targetTicks = (int) (relativeTargetAngle * ticksPerDegreeTurret);
            turret.setTargetPosition(targetTicks);

            // Telemetry
            telemetry.addData("Robot Heading", "%.2f", robotHeading);
            telemetry.addData("Turret Field Target", "%.2f", turretFieldHeading);
            telemetry.addData("Turret Relative Angle", "%.2f", relativeTargetAngle);
            telemetry.addData("Turret Target Ticks", targetTicks);
            telemetry.addData("Turret Current Ticks", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}
