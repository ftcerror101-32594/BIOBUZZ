package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.DiagnosticLogger;

import java.io.IOException;

@TeleOp(group = "Robot Centric")
public class TeleOpMecanumRobot2Drivers extends LinearOpMode {
    private IMU.Parameters parameters;
    @Override
    public void runOpMode() throws InterruptedException {
        // CPR of a Rev HD Hex Motor.
        final int CPR = 28;
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        DcMotorEx bigHooperMotor = hardwareMap.get(DcMotorEx.class, "bigHooperMotor");
        DcMotorEx smallHooperMotor = hardwareMap.get(DcMotorEx.class, "smallHooperMotor");

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bigHooperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(BRAKE);
        frontRightMotor.setZeroPowerBehavior(BRAKE);
        backRightMotor.setZeroPowerBehavior(BRAKE);
        backLeftMotor.setZeroPowerBehavior(BRAKE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // The private parameters variable is passed to the logger at the bottom of this class.
        this.parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo UP / USB FORWARD
        imu.initialize(parameters);

        DiagnosticLogger logger = getLogger();
        Thread loggerRuntimeThread = new Thread(logger);
        logger.resumeRun();
        loggerRuntimeThread.start();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double brakePower = 1 - gamepad1.right_trigger;
            double bigHooperPower = gamepad2.left_trigger * 6000;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            telemetry.addData("1, LT", gamepad1.left_trigger);
            telemetry.addData("1, RT", gamepad1.right_trigger);
            telemetry.addData("1, LB", gamepad1.left_bumper);
            telemetry.addData("1, LB", gamepad1.right_bumper);
            telemetry.addData("1, LT", gamepad1.left_trigger);
            telemetry.addData("1, LS-X", gamepad1.left_stick_x);
            telemetry.addData("1, LS-Y", gamepad1.left_stick_y);
            telemetry.addData("1, RS-X", gamepad1.right_stick_x);
            telemetry.addData("1, RS-Y", gamepad1.right_stick_y);
            telemetry.addData("1, DU", gamepad1.dpad_up);
            telemetry.addData("1, DL", gamepad1.dpad_left);
            telemetry.addData("1, DD", gamepad1.dpad_down);
            telemetry.addData("1, DR", gamepad1.dpad_right);
            telemetry.addData("1, A", gamepad1.a);
            telemetry.addData("1, B", gamepad1.b);
            telemetry.addData("1, X", gamepad1.x);
            telemetry.addData("1, Y", gamepad1.y);

            telemetry.addData("2, LT", gamepad2.left_trigger);
            telemetry.addData("2, RT", gamepad2.right_trigger);
            telemetry.addData("2, LB", gamepad2.left_bumper);
            telemetry.addData("2, LB", gamepad2.right_bumper);
            telemetry.addData("2, LT", gamepad2.left_trigger);
            telemetry.addData("2, LS-X", gamepad2.left_stick_x);
            telemetry.addData("2, LS-Y", gamepad2.left_stick_y);
            telemetry.addData("2, RS-X", gamepad2.right_stick_x);
            telemetry.addData("2, RS-Y", gamepad2.right_stick_y);
            telemetry.addData("2, DU", gamepad2.dpad_up);
            telemetry.addData("2, DL", gamepad2.dpad_left);
            telemetry.addData("2, DD", gamepad2.dpad_down);
            telemetry.addData("2, DR", gamepad2.dpad_right);
            telemetry.addData("2, A", gamepad2.a);
            telemetry.addData("2, B", gamepad2.b);
            telemetry.addData("2, X", gamepad2.x);
            telemetry.addData("2, Y", gamepad2.y);

            telemetry.update();

            if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y)
            {
                logger.stopRun();
            }

            if(gamepad2.y)
            {
                bigHooperMotor.setPower(-0.9);
            } else
            {
                bigHooperMotor.setVelocity((bigHooperPower / 60)*CPR);
            }

            if (gamepad2.a)
            {
                smallHooperMotor.setPower(0.9);
            } else if (gamepad2.x) {
                smallHooperMotor.setPower(-0.9);
            } else
            {
                smallHooperMotor.setPower(0);
            }

            frontLeftMotor.setPower(frontLeftPower * brakePower);
            backLeftMotor.setPower(backLeftPower * brakePower);
            frontRightMotor.setPower(frontRightPower * brakePower);
            backRightMotor.setPower(backRightPower * brakePower);
        }
    }

    @NonNull
    private DiagnosticLogger getLogger()
    {
        DiagnosticLogger logger;
        try
        {
            logger = new DiagnosticLogger(
                    telemetry, hardwareMap,
                    new String[]
                            {
                                    "frontLeftMotor", "backLeftMotor",
                                    "frontRightMotor", "backRightMotor",
                                    "bigHooperMotor", "smallHooperMotor"
                            },
                    null, null, "imu", parameters
            );
        } catch (IOException e)
        {
            telemetry.addData("IOException", e.getMessage());
            throw new RuntimeException(e);
        }
        return logger;
}
}
