package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmode.teleop.Flywheel;

@Autonomous(name = "Shoot4")
public class Shoot4 extends AutoHardware {
    private static final double TARGET_RPM = 3500.0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.get(DcMotor.class, "in");
        intake.setPower(0.0);
        DcMotor transfer = hardwareMap.get(DcMotor.class, "trans");
        transfer.setPower(0.0);
        intake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initFlywheel();

        try {
            setFlywheelTargetRpm(TARGET_RPM);
            telemetry.addData("Status", "Shoot4 ready; press Play");
            telemetry.addData("Flywheel Target (RPM)", "%.0f", TARGET_RPM);
            telemetry.update();
            waitForStart();
            if (!opModeIsActive()) return;

            telemetry.addData("Shoot4 Phase", "Spin up");
            if (!startFlywheel() || !waitForFlywheelReady()) return;

            // Run both feeders forward for 5 seconds.
            telemetry.addData("Shoot4 Phase", "Continuous feed 1");
            intake.setPower(Flywheel.Config.INTAKE_FORWARD_POWER);
            transfer.setPower(Flywheel.Config.TRANSFER_FORWARD_POWER);
            for (int i = 0; i < 50 && opModeIsActive(); i++) {
                updateFlywheel();
                telemetry.update();
                sleep(100);
            }
            if (!opModeIsActive()) return;

            // Five pulses: 5 x 100 ms ON, then 5 x 100 ms OFF per pulse.
            telemetry.addData("Shoot4 Phase", "Pulsed feed");
            for (int i = 0; i < 50 && opModeIsActive(); i++) {
                boolean on = i % 10 < 5;
                intake.setPower(on ? Flywheel.Config.INTAKE_FORWARD_POWER : 0.0);
                transfer.setPower(on ? Flywheel.Config.TRANSFER_FORWARD_POWER : 0.0);
                updateFlywheel();
                telemetry.update();
                sleep(100);
            }
            if (!opModeIsActive()) return;

            // Run both feeders forward for another 5 seconds.
            telemetry.addData("Shoot4 Phase", "Continuous feed 2");
            intake.setPower(Flywheel.Config.INTAKE_FORWARD_POWER);
            transfer.setPower(Flywheel.Config.TRANSFER_FORWARD_POWER);
            for (int i = 0; i < 50 && opModeIsActive(); i++) {
                updateFlywheel();
                telemetry.update();
                sleep(100);
            }
        } finally {
            intake.setPower(0.0);
            transfer.setPower(0.0);
            stopFlywheel();
        }
    }
}
