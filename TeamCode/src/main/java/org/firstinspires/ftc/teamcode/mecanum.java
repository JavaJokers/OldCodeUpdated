package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class mecanum extends LinearOpMode {
    double grabberPos = 0;
    int IMUResets = 0;

    public void runOpMode() throws InterruptedException {





        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("front_left");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("back_left");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("front_right");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("back_right");
        DcMotor arm1 = hardwareMap.dcMotor.get("arm1");
        DcMotor duckies = hardwareMap.dcMotor.get("duckies");
        Servo grabber = hardwareMap.servo.get("grabber");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //set brake mode
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad1.right_bumper) {
                motorFrontLeft.setPower(frontLeftPower * 0.25);
                motorFrontRight.setPower(frontRightPower * 0.25);
                motorBackLeft.setPower(backLeftPower * 0.25);
                motorBackRight.setPower(backRightPower * 0.25);

            } else if (gamepad1.left_bumper) {
                motorFrontLeft.setPower(frontLeftPower * 0.25);
                motorFrontRight.setPower(frontRightPower * 0.25);
                motorBackLeft.setPower(backLeftPower * 0.25);
                motorBackRight.setPower(backRightPower * 0.25);

            } else {
                motorFrontLeft.setPower(frontLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackLeft.setPower(backLeftPower);
                motorBackRight.setPower(backRightPower);

            }


            if(gamepad1.dpad_left){
                grabberPos -= 0.01;
            }else if(gamepad1.dpad_right){
                grabberPos += 0.01;
            }

            if(grabberPos > 1){
                grabberPos = 1;
            } else if(grabberPos < 0){
                grabberPos = 0;
            }

            grabber.setPosition(grabberPos);

            if(gamepad1.right_trigger == 1 && gamepad1.left_trigger == 1){
                imu.initialize(parameters);
                IMUResets++;
            }

            if(gamepad1.dpad_up){
                arm1.setPower(0.5);
            } else if(gamepad1.dpad_down){
                arm1.setPower(-0.1);
            }else{
                arm1.setPower(0.1);
            }


            if (gamepad1.x) {
                duckies.setPower(-0.8);
            } else {
                duckies.setPower(0);
            }

            // Show the wheel power.
            telemetry.addData("IMU Resets", IMUResets);
            telemetry.addData("Grabber", grabberPos);
            telemetry.addData("Motors", "carousel");
            telemetry.update();

        }
    }
}