package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.opmode.autonomous.AutoHardware;

@Disabled
@TeleOp(name="Gyro Test", group = "test")
public class GyroTest extends AutoHardware {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // imu variables
    private YawPitchRollAngles lastAngles;
    private double currAngle = 0.0;


    @Override
    public void runOpMode() {
        initAll();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()){
            // telemetry prints out information on drivers station
            telemetry.addData("Current Yaw: ", getCurrentYaw());
            telemetry.addData("TicksPerInc", TICKS_PER_INCH);
            telemetry.update();
       }

    }
    public void resetAngle(){
        lastAngles = imu.getRobotYawPitchRollAngles();
        currAngle = 0;
    }

    public double getAngle(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double deltaAngle = orientation.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle>180){
            deltaAngle -= 360;
        }
        else if (deltaAngle<=-180){
            deltaAngle+=360;
        }

        currAngle+=deltaAngle;
        lastAngles = orientation;

        telemetry.addData("gyro", orientation.getYaw(AngleUnit.DEGREES));
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error)>2){
            //double motorPower = (error < 0 ? -0.3 : 0.3);
            double motorPower = Math.min(error*0.01,0.3);
            //robot.setDrivePower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        //setAllDrivePower(0);
    }
    public void turnTo(double degrees){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double error = degrees - orientation.getYaw(AngleUnit.DEGREES);

        if (error > 180){
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }
        while (opModeIsActive() && Math.abs(error)>10){
            YawPitchRollAngles currOrientation = imu.getRobotYawPitchRollAngles();

            double motorPower = (error < 0 ? -0.35 : 0.35);
            //setDrivePower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - currOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Orientation: ",currOrientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Error: ", Math.abs(error));

            telemetry.update();

        }

    }
}

