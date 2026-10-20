package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware4;

import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

/** Encoder-regulated flywheel. Call update(gamepad) once per OpMode loop. */
public class Flywheel {
    /** All mechanism controls, powers, and flywheel tuning settings belong here. */
    public static final class Config {
        // RPM is measured at the flywheel. The existing 6000 RPM, 1:1 motor uses 28 ticks/rev.
        // Change this value if the motor or gearing changes.
        public static final double ENCODER_TICKS_PER_FLYWHEEL_REV = 28.0;
        public static final double DEFAULT_RPM = 5100.0;
        public static final double NEAR_RPM = 5100.0;
        public static final double FAR_RPM = 5350.0;
        public static final double MAX_RPM = 6000.0;
        public static final double RPM_STEP = 100.0;

        // Report READY after the wheel stays inside this speed band continuously.
        public static final double READY_TOLERANCE_RPM = 80.0;
        public static final double READY_STABLE_SECONDS = 0.15;

        // Existing PIDF values are retained as a starting point; tune on the real robot.
        public static final double PIDF_P = 350.0;
        public static final double PIDF_I = 0.1;
        public static final double PIDF_D = 0.0;
        public static final double PIDF_F = 12.0;
        public static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
        public static final DcMotor.ZeroPowerBehavior STOP_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

        // Signed powers preserve the robot's existing intake/transport forward directions.
        public static final double INTAKE_FORWARD_POWER = -1.0;
        public static final double INTAKE_REVERSE_POWER = 1.0;
        public static final double TRANSFER_FORWARD_POWER = 1.0;
        public static final double TRANSFER_REVERSE_POWER = -1.0;

        // Edit the gamepad field on the right to change a control mapping.
        public static final double TRIGGER_THRESHOLD = 0.05;
        public static final ToDoubleFunction<Gamepad> INTAKE_TRIGGER = g -> g.left_trigger;
        public static final ToDoubleFunction<Gamepad> TRANSFER_TRIGGER = g -> g.right_trigger;
        public static final Predicate<Gamepad> REVERSE_BOTH = g -> g.left_bumper;
        public static final Predicate<Gamepad> TOGGLE = g -> g.right_bumper;
        public static final Predicate<Gamepad> FASTER = g -> g.dpad_up;
        public static final Predicate<Gamepad> SLOWER = g -> g.dpad_down;
        public static final Predicate<Gamepad> NEAR_PRESET = g -> g.x;
        public static final Predicate<Gamepad> FAR_PRESET = g -> g.b;

        private Config() {}
    }

    public enum State { OFF, SPINNING_UP, READY }

    private final DcMotorEx motor;
    private final DoubleSupplier seconds;
    private double targetRpm = clampRpm(Config.DEFAULT_RPM);
    private double actualRpm;
    private boolean enabled;
    private State state = State.OFF;
    private double inToleranceSince = Double.NaN;
    private boolean toggleWasPressed;
    private int previousAdjustment;

    /** RobotHardware4.init(hardwareMap) must have been called before constructing this. */
    public Flywheel(RobotHardware4 robot) {
        this(Objects.requireNonNull(robot, "robot").hogback,
                () -> System.nanoTime() / 1_000_000_000.0);
    }

    // Inject a motor and monotonic clock for tests without operating real hardware.
    Flywheel(DcMotorEx motor, DoubleSupplier seconds) {
        this.motor = Objects.requireNonNull(motor, "Initialize RobotHardware4 before Flywheel");
        this.seconds = Objects.requireNonNull(seconds, "seconds");
        motor.setPower(0.0);
        motor.setDirection(Config.MOTOR_DIRECTION);
        motor.setZeroPowerBehavior(Config.STOP_BEHAVIOR);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(Config.PIDF_P, Config.PIDF_I,
                Config.PIDF_D, Config.PIDF_F);
        stop();
    }

    /** Toggle and RPM adjustments happen once per button press, not once per loop. */
    public void update(Gamepad gamepad) {
        boolean togglePressed = Config.TOGGLE.test(gamepad);
        if (togglePressed && !toggleWasPressed) {
            if (isRunning()) {
                stop();
            } else {
                start();
            }
        }
        toggleWasPressed = togglePressed;

        if (Config.FAR_PRESET.test(gamepad)) {
            setTargetRpm(Config.FAR_RPM);
            previousAdjustment = 0;
        } else if (Config.NEAR_PRESET.test(gamepad)) {
            setTargetRpm(Config.NEAR_RPM);
            previousAdjustment = 0;
        } else {
            int adjustment = (Config.FASTER.test(gamepad) ? 1 : 0)
                    - (Config.SLOWER.test(gamepad) ? 1 : 0);
            if (adjustment != 0 && adjustment != previousAdjustment) {
                setTargetRpm(targetRpm + adjustment * Config.RPM_STEP);
            }
            previousAdjustment = adjustment;
        }
        update();
    }

    /** Refresh the motor command and readiness. Also usable without gamepad controls. */
    public void update() {
        actualRpm = motor.getVelocity() * 60.0 / Config.ENCODER_TICKS_PER_FLYWHEEL_REV;
        if (!isRunning()) {
            stop();
            return;
        }

        motor.setVelocity(targetRpm * Config.ENCODER_TICKS_PER_FLYWHEEL_REV / 60.0);
        if (Double.isFinite(actualRpm)
                && Math.abs(targetRpm - actualRpm) <= Config.READY_TOLERANCE_RPM) {
            double now = seconds.getAsDouble();
            if (Double.isNaN(inToleranceSince)) {
                inToleranceSince = now;
            }
            state = now - inToleranceSince >= Config.READY_STABLE_SECONDS
                    ? State.READY : State.SPINNING_UP;
        } else {
            invalidateReadiness();
        }
    }

    public void start() {
        if (!enabled) {
            enabled = true;
            invalidateReadiness();
        }
    }

    /** Stop commands zero output and lets the wheel coast with FLOAT. */
    public void stop() {
        enabled = false;
        state = State.OFF;
        inToleranceSince = Double.NaN;
        motor.setVelocity(0.0);
        motor.setPower(0.0);
    }

    public void setTargetRpm(double rpm) {
        if (!Double.isFinite(rpm)) {
            throw new IllegalArgumentException("Flywheel RPM must be finite");
        }
        double newTarget = clampRpm(rpm);
        if (newTarget != targetRpm) {
            targetRpm = newTarget;
            invalidateReadiness();
            if (targetRpm == 0.0) {
                stop();
            }
        }
    }

    public boolean isRunning() { return enabled && targetRpm > 0.0; }
    public boolean isReady() { return state == State.READY; }
    public State getState() { return state; }
    public double getTargetRpm() { return targetRpm; }
    public double getActualRpm() { return actualRpm; }
    public double getErrorRpm() { return targetRpm - actualRpm; }

    private void invalidateReadiness() {
        inToleranceSince = Double.NaN;
        state = isRunning() ? State.SPINNING_UP : State.OFF;
    }

    private static double clampRpm(double rpm) {
        return Math.max(0.0, Math.min(Config.MAX_RPM, rpm));
    }
}
