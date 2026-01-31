package org.firstinspires.ftc.teamcode.opmode.autonomous;
//test


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//ignore this for now
@Autonomous(name="Blue_Far_3_Leave_Base")
public class Blue_Far_3_Leave_Base extends AutoHardware  {

    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;
    double hogback_rpm_far = 5350;  // Rotations Per Minute

    public Blue_Far_3_Leave_Base() {
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initAll();

        //reset encoder
        setAutoDriveMotorMode();
        // Set target rpm based on the specific robot position: far, near
        setHogbackTargetRpm(hogback_rpm_far);

        telemetry.update();
        waitForStart();

        // Turn intake subsystem so that intake system is on and 2nd & 3rd preloaded balls can be
        // pushed to the triggering position
        turnOnIntakeSubsystem();
        // Use state machine to shot 3 balls consecutively.
        triggerShoot(3);
        turnOffIntakeSubsystem();

        double targetYaw = getCurrentYaw();
        double inchesForward = 16;
        int ticksForward = (int)(inchesForward * TICKS_PER_INCH);
        driveMotors(ticksForward, ticksForward, ticksForward, ticksForward,
                0.5,
                true,
                targetYaw);
        sleep(30000);
    }
}
