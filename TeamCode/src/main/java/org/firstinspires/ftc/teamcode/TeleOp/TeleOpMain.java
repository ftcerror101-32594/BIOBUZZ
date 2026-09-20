// This is the main java class for the TeleOp controller-based code of the robot
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Main")
public class TeleOpMain extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Robot Initialized:", "True");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Sets up joystick & corr. motor movement for DRIVE controller

            double leftJoystickX = gamepad1.left_stick_x;
            double leftJoystickY = gamepad1.left_stick_y;
            double rightJoystickX = gamepad1.right_stick_x;

            // debug statements to report current joystick push headings
            telemetry.addData("Left Joystick X:", leftJoystickX);
            telemetry.addData("Left Joystick Y:", leftJoystickY);
            telemetry.addData("Right Joystick X:", rightJoystickX);

            leftJoystickY = -(leftJoystickY); // invert y-axis (forward= +, backward= -)

            if (Math.abs(leftJoystickX) < 0.10) { // create deadzone for x-axis
                leftJoystickX = 0;
            }
            if (Math.abs(leftJoystickY) < 0.10) { // create deadzone for y-axis
                leftJoystickY = 0;
            }
            if (Math.abs(rightJoystickX) < 0.10) { // create deadzone for x-axis
                rightJoystickX = 0;
            }

            double defaultSpeed = 0.75;
            double motorPower = leftJoystickY * defaultSpeed;
            double strafePower = leftJoystickX * defaultSpeed;
            double turnPower = rightJoystickX * defaultSpeed;

            // motor power (straight) stays same; aka constant
            // left = +turn ; right = -turn
            // strafe: diagonals are same, RF & LR = -, LF & RR = +
            double leftFrontSpeed = motorPower + strafePower + turnPower;
            double rightFrontSpeed = motorPower - strafePower - turnPower;
            double leftRearSpeed = motorPower - strafePower + turnPower;
            double rightRearSpeed = motorPower + strafePower - turnPower;

            // debug statements to report final motor powers
            telemetry.addData("Left Front Speed", leftFrontSpeed);
            telemetry.addData("Right Front Speed", rightFrontSpeed);
            telemetry.addData("Left Rear Speed", leftRearSpeed);
            telemetry.addData("Right Rear Speed", rightRearSpeed);

            //String[] motorArray = {"Left Front", "Right Front",
                                    //"Left Rear", "Right Rear"};

            //double[] motorSpeedArray = {leftFrontSpeed, rightFrontSpeed,
                                        //leftRearSpeed, rightRearSpeed};

            // Compares max powers to see which motor requires most speed
            double maxPower = Math.max(1.0, Math.max(
                    Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed)),
                    Math.max(Math.abs(leftRearSpeed), Math.abs(rightRearSpeed))
            ));

            // Proportionately scale down in case -1 <= power >= 1
            leftFrontSpeed /= maxPower;
            rightFrontSpeed /= maxPower;
            leftRearSpeed /= maxPower;
            rightRearSpeed /= maxPower;

            // Set the finalized speeds to the respective motors
            robot.leftFront.setPower(leftFrontSpeed);
            robot.rightFront.setPower(rightFrontSpeed);
            robot.leftRear.setPower(leftRearSpeed);
            robot.rightRear.setPower(rightRearSpeed);
        }

    }
}
