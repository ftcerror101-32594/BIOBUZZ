package org.firstinspires.ftc.teamcode.TeleOp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DiagnosticLogger;

import java.io.IOException;
import java.util.List;
import java.util.Locale;

@TeleOp(group = "Field Centric")
public class TeleOpMecanumField2Drivers extends LinearOpMode
{
    private final static int CPR = Constants.CPR;
    private final static double LOCK_ON_DENOMINATOR = Constants.LOCK_ON_DENOMINATOR;
    private static IMU.Parameters parameters;

    private final static FtcDashboard DASH = FtcDashboard.getInstance();

    private final static TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode()
    {
        Telemetry telemetryD = DASH.getTelemetry();
        Telemetry telemetryM = new MultipleTelemetry(telemetry, DASH.getTelemetry());

        telemetryD.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetryM.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Note to self: only use hardwareMap in or after the initialization!
        DcMotorEx[] base = Constants.getDriveMotors(hardwareMap);
        DcMotorEx[] scoring = Constants.getScoringMotors(hardwareMap);
        IMU imu = Constants.getIMU(hardwareMap);
        Limelight3A limelight = Constants.getLimelight(hardwareMap);

        parameters = Constants.getIMUParms();
        imu.initialize(parameters);

        limelight.start();

        boolean lockOn = false;

        telemetryM.addLine("> Ready.");
        telemetryM.update();

        waitForStart();

        // Ensure telemetry is clear on start.
        // FtcDashboard tends to keep telemetry in the buffer even after
        // update()-ing.
        // (We abuse this a bit later to have nicely formatted text, since
        // FtcDashboard also likes alphabetizing data.)
        telemetryM.clear();

        DiagnosticLogger logger = getLogger();
        Thread loggerRuntimeThread = new Thread(logger);
        logger.resumeRun();
        loggerRuntimeThread.start();

        boolean pastLeftBumper = false;

        while(opModeIsActive())
        {
            LLResult rawResult = limelight.getLatestResult();
            List<LLResultTypes.FiducialResult> fiducial = rawResult.getFiducialResults();
            boolean resultIsValid = rawResult.isValid();

            boolean leftBumper = gamepad1.left_bumper;
            double targetX;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*1.1;
            double rx;
            if(lockOn && resultIsValid)
            {
                targetX = rawResult.getTx();
                // A simple proportional input; May use PI (proportional-integral)
                // control in the future.
                // getTx returns a value in the range of -27.25 to 27.25.
                // We want to get a value from -1 to 1, so we can just divide
                // by 27.25. (27.25/27.25 = 1)
                rx = targetX/LOCK_ON_DENOMINATOR;
            } else
            {
                rx = gamepad1.right_stick_x;
            }

            double brakePower = 1-gamepad1.right_trigger;
            // Max target RPM: 4900
            double bigHooperPower = 2500 + (gamepad2.left_trigger * 2400);

            // --- Trigonometry Warning !!! --- (Continues for 14 lines)
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeadingDegrees = botHeading * (180.0/Math.PI);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // -- Runtime Telemetry --
            double bigHooperSpeed = (scoring[0].getVelocity()*60)/CPR;
            String runtimeSeconds = String.format(
                    Locale.ENGLISH, "%.4f", getRuntime()
            );

            packet.clearLines();
            packet.put("<span style=display:none;>",
                    "</span>" +
                            "Runtime (Seconds): " + runtimeSeconds + "<br>" +
                            "<br>" +
                            "== Shooting Motor (scoring[0]) =="
            );

            // runtimeSeconds shouldn't be graphable,so we make it a line
            // instead of data. (FtcDashboard graph already adds time as X)
            DASH.sendTelemetryPacket(packet);
            telemetryD.addData("shooting-input-rpm", bigHooperPower);
            telemetryD.addData("shooting-speed-rpm", bigHooperSpeed);
            telemetryD.update();
            telemetryD.addData("<br><span style=display:none;>", "</span>");
            telemetryD.update();
            telemetryD.addData("right-stick-x", rx);
            telemetryD.addData("bot-heading", botHeadingDegrees);
            telemetryD.addData("lock-on", lockOn);
            telemetryD.addData("valid-target", resultIsValid);
            telemetryD.update();

            telemetry.addData("Runtime (Seconds)", runtimeSeconds);
            telemetry.addLine();
            telemetry.addLine("== Shooting Motor (scoring[0]) ==");
            telemetry.addData("shooting-input-rpm", bigHooperPower);
            telemetry.addData("shooting-speed-rpm", bigHooperSpeed);
            telemetry.addLine();
            telemetry.addData("right-stick-x", rx);
            telemetry.addData("bot-heading", botHeadingDegrees);
            telemetry.addData("lock-on", lockOn);
            telemetry.addData("valid-target", resultIsValid);
            telemetry.update();

            if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y)
            {
                logger.stopRun();
            }

            if (gamepad1.start)
            {
                imu.resetYaw();
            }

            // pastLeftBumper initializes to false
            if(leftBumper && ! pastLeftBumper)
            {
                // LockOn toggle
                lockOn = !lockOn;

            }

            if(gamepad2.y)
            {
                scoring[0].setPower(-0.9);
            } else
            {
                scoring[0].setVelocity((bigHooperPower / 60)*CPR);
            }

            if (gamepad2.a)
            {
                scoring[1].setPower(0.9);
            }  else if (gamepad2.x)
            {
                scoring[1].setPower(-0.9);
            } else
            {
                scoring[1].setPower(0);
            }

            base[0].setPower(frontLeftPower*brakePower);
            base[1].setPower(backLeftPower*brakePower);
            base[2].setPower(backRightPower*brakePower);
            base[3].setPower(frontRightPower*brakePower);

            pastLeftBumper = leftBumper;
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
                    null,
                    new String[]
                    {
                            "frontLeftMotor", "backLeftMotor",
                            "frontRightMotor", "backRightMotor",
                            "bigHooperMotor", "smallHooperMotor"
                    },
                    null, "imu", parameters
            );
        } catch (IOException e)
        {
            throw new RuntimeException(e);
        }
        return logger;
    }
}