package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware4;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {
    private final RobotHardware4 robot = new RobotHardware4();
    private Flywheel flywheel;

    @Override
    public void init() {
        robot.init(hardwareMap);
        flywheel = new Flywheel(robot);
        robot.transport.setPower(0.0);
        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(11);
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

        if (robot.motorfl == null || robot.motorfr == null || robot.motorbl == null || robot.motorbr == null) {
            telemetry.addData("DriveMotorNull", String.format("lf:%b rf:%b lb:%b rb:%b",
                    robot.motorfl == null, robot.motorfr == null, robot.motorbl == null, robot.motorbr == null));
            telemetry.update();
        } else {
            robot.motorfl.setPower(flPower);
            robot.motorfr.setPower(frPower);
            robot.motorbl.setPower(blPower);
            robot.motorbr.setPower(brPower);
        }


        flywheel.update(gamepad1);
        // Forward controls are independent. LB overrides both with full reverse power.
        robot.intake.setPower(intakePower(gamepad1));
        robot.transport.setPower(transferPower(gamepad1));

        telemetry.addData("Flywheel State", flywheel.getState());
        telemetry.addData("Flywheel Target (RPM)", "%.0f", flywheel.getTargetRpm());
        telemetry.addData("Flywheel Actual (RPM)", "%.0f", flywheel.getActualRpm());
        telemetry.addData("Flywheel Error (RPM)", "%.0f", flywheel.getErrorRpm());
        telemetry.update();
    }

    static double intakePower(Gamepad gamepad) {
        if (Flywheel.Config.REVERSE_BOTH.test(gamepad)) {
            return Flywheel.Config.INTAKE_REVERSE_POWER;
        }
        return Flywheel.Config.INTAKE_FORWARD_POWER
                * triggerPower(Flywheel.Config.INTAKE_TRIGGER.applyAsDouble(gamepad));
    }

    static double transferPower(Gamepad gamepad) {
        if (Flywheel.Config.REVERSE_BOTH.test(gamepad)) {
            return Flywheel.Config.TRANSFER_REVERSE_POWER;
        }
        return Flywheel.Config.TRANSFER_FORWARD_POWER
                * triggerPower(Flywheel.Config.TRANSFER_TRIGGER.applyAsDouble(gamepad));
    }

    private static double triggerPower(double value) {
        double clipped = Math.max(0.0, Math.min(1.0, value));
        return clipped > Flywheel.Config.TRIGGER_THRESHOLD ? clipped : 0.0;
    }

    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.stop();
        }
        if (robot.intake != null) {
            robot.intake.setPower(0.0);
        }
        if (robot.transport != null) {
            robot.transport.setPower(0.0);
        }
    }
}
