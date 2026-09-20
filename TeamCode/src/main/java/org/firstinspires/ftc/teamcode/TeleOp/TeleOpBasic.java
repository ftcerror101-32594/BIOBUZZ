package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpBasic extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontMotor = hardwareMap.dcMotor.get("frontMotor");
        DcMotor bigHooperMotor = hardwareMap.dcMotor.get("bigHooperMotor");
        DcMotor backMotor = hardwareMap.dcMotor.get("backMotor");
        DcMotor smallHooperMotor = hardwareMap.dcMotor.get("smallHooperMotor");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Bad teleop code for bad no no wheels
            boolean shoot= gamepad1.aWasPressed();
            double leftPower = gamepad1.left_stick_y;
            double rightPower = gamepad1.right_stick_y;

            frontMotor.setPower(leftPower);
            backMotor.setPower(rightPower);
            if (gamepad1.a) {
                bigHooperMotor.setPower(-1);
                smallHooperMotor.setPower(1);
            } else{
                bigHooperMotor.setPower(0);
                smallHooperMotor.setPower(0);
            }
                
            if (gamepad1.b) {
                smallHooperMotor.setPower(0);
                frontMotor.setPower(0);
                backMotor.setPower(0);
            }
        }
    }
}