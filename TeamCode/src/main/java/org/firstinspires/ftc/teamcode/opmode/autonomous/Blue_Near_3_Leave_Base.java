package org.firstinspires.ftc.teamcode.opmode.autonomous;
//test


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//ignore this for now
@Autonomous(name="Blue_Near_3_Leave_Base")
public class Blue_Near_3_Leave_Base extends AutoHardware  {

    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;
    double hogback_rpm_near = 5100;  // Rotations Per Minute

    public Blue_Near_3_Leave_Base() {
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initAll();

        //reset encoder
        setAutoDriveMotorMode();
        // Set target rpm based on the specific robot position: far, near
        setHogbackTargetRpm(hogback_rpm_near);

        telemetry.update();
        waitForStart();
        double targetYaw = getCurrentYaw();

        double inchesBackward = -45;
        int ticksBackward = (int)(inchesBackward * TICKS_PER_INCH);
        driveMotors(ticksBackward, ticksBackward, ticksBackward, ticksBackward,
                0.4,
                false,
                targetYaw);

        // Turn intake subsystem so that intake system is on and 2nd & 3rd preloaded balls can be
        // pushed to the triggering position
        turnOnIntakeSubsystem();
        // Use state machine to shot 3 balls consecutively.
        triggerShoot(3);
        turnOffIntakeSubsystem();

        double inchesForward = 24;
        targetYaw = -45;
        turnToTargetYaw(targetYaw, 0.5, 3000);
        int ticksMove = (int)(inchesForward * TICKS_PER_INCH);
        driveMotors(ticksMove, ticksMove, ticksMove, ticksMove,
                0.5,
                true,
                targetYaw);
        sleep(30000);
    }
}
