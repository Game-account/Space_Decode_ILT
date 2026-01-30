package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//test github permission
@TeleOp(name = "OpMode3")
@Disabled
public class OpMode3 extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private int sleepMs1 = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * 0.5;
            double vertical = -gamepad1.left_stick_y * 0.5;
            double turn = gamepad1.right_stick_x * 0.5;

            robot.setDrivePower(vertical+turn-horizontal,vertical-turn+horizontal,vertical+turn+horizontal,vertical-turn-horizontal);


            if (-gamepad2.left_stick_y > 0.1){
                robot.setArmPower(0.25);
            }
            else if (-gamepad2.left_stick_y < -0.1){
                robot.setArmPower(-0.25);
            }
            else{
                robot.setArmPower(0);
            }

            //
            //lift arm start
            if(gamepad1.a) { //if button a pressed
                TiltLiftOne(-0.5, 300, -0.1, 0.8, 350, 0);

                //tilt the lift to be upright
                robot.liftHex.setPower(-0.5);   //set motor power
                sleep(300);             // let motor run for some time seconds.
                robot.liftHex.setPower(-0.1);   //set lower motor power to maintain the position

                // Extend liftArm
                robot.liftArm.setPower(0.8);
                sleep(350);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }

            if (gamepad1.b) {
                robot.liftHex.setPower(0.9);   //set motor power
                sleep(300);             // let motor run for some time seconds.
                robot.liftHex.setPower(0.4);   //set lower motor power to maintain the position

                // Retract liftArm
                robot.liftArm.setPower(-1.0);
                sleep(350);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }

            if(gamepad1.y) { //if button a pressed
                // Extend liftArm
                robot.liftArm.setPower(0.8);
                sleep(300);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }
            if(gamepad1.x) { //if button a pressed
                // Retract liftArm
                robot.liftArm.setPower(-1.0);
                sleep(300);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }


            // Launch airplane
            if (gamepad2.a) {
                // Tilt the launcher in a given degree in order to launch airplane over the bar
                TiltLiftOne(-0.5, (int)(300 * 1.5), -0.1, 0.8, (int)(350 * 1.6), 0);

                robot.launcher.setPower(-1.0);
                sleep(3000);
                robot.airplaneFeeder.setPosition(0);
                sleep(1000);
                robot.launcher.setPower(0.0);
                robot.airplaneFeeder.setPosition(0.5);
            }

           if (-gamepad2.left_stick_y > 0.1){
                robot.setArmPower(0.45);
            }
            else if (-gamepad2.left_stick_y < -0.1){
                robot.setArmPower(-0.45);
            }
            else{
                robot.setArmPower(0);
            }

            //servos

//grabber
            if (gamepad2.left_trigger > 0.5) {
                robot.grabServo.setPosition(0.4); // open
            }
            else if (gamepad2.right_trigger>0.5){
                robot.grabServo.setPosition(1); // close
            }


//tilt servo
            double initTilt = 0.0;

            if (gamepad2.right_stick_y > 0.7) {
                initTilt += 0.1;

                if (initTilt > 1.0) {
                    initTilt = 1.0;
                    robot.tiltServo.setPosition(-initTilt);
                }

                else robot.tiltServo.setPosition(-initTilt);
            }

            if (gamepad2.right_stick_y < -0.7) {
                initTilt -= 0.1;

                if (initTilt < -1.0) {
                    initTilt = -1.0;
                    robot.tiltServo.setPosition(initTilt);
                }

                else robot.tiltServo.setPosition(initTilt);


            }



            telemetry.update();


        }
    }

    private void TiltLiftOne(double crankPowerBegin, int crankTimeMs, double crankPowerEnd,
                             double liftPowerBegin, int liftTimeMs, double liftPowerEnd) {
        //tilt the lift to be upright
        robot.liftHex.setPower(crankPowerBegin);   //set motor power
        sleep(crankTimeMs);          // let motor run for some time seconds.
        robot.liftHex.setPower(crankPowerEnd);   //set lower motor power to maintain the position

        // Extend liftArm
        robot.liftArm.setPower(liftPowerBegin);
        sleep(liftTimeMs);             // let motor run for some time seconds.
        robot.liftArm.setPower(liftPowerEnd);
    }

}

