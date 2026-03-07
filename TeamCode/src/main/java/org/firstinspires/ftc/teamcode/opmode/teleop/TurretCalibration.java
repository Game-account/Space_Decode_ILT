package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Turret Calibration", group = "Calibration")
@Disabled
public class TurretCalibration extends LinearOpMode {

    private DcMotor turret = null;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(DcMotor.class, "Turret");
        
        // Reset encoder to start at 0
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Turret encoder reset.");
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Align turret exactly forward manually.");
        telemetry.addLine("2. Press Play.");
        telemetry.addLine("3. Use Left Stick to move turret to exactly 90 degrees.");
        telemetry.addLine("4. Note the 'Ticks Per Degree' value.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Manual control to find physical 90 degrees
            double power = -gamepad1.left_stick_x * 0.3; // Low power for precision
            turret.setPower(power);

            int currentTicks = turret.getCurrentPosition();
            double ticksPerDegree = Math.abs(currentTicks) / 90.0;

            telemetry.addData("Current Ticks", currentTicks);
            telemetry.addData("If at 90 deg, TicksPerDegree is", "%.4f", ticksPerDegree);
            telemetry.addLine();
            telemetry.addLine("Controls:");
            telemetry.addLine("Left Stick X - Move Turret");
            telemetry.addLine("Button A - Reset Encoder at current position");
            
            if (gamepad1.a) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.update();
        }
    }
}
