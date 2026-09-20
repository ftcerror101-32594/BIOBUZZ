// RobotHardware class essentially describes/states inventory and parts of robot
// RobotHardware class also acts as a library where changes can update to all classes
package org.firstinspires.ftc.teamcode;

// DC Motor Extension allows for advanced controls such as velocity control
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {
    // Setting up the DC Motors of the robot
    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;

    // Describes the hardware specifics of the robot and initializes them
    public void init(HardwareMap hardwareMap) {
        // Setting the variables and shortened terms as basically, an ID
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LR");
        rightRear = hardwareMap.get(DcMotorEx.class, "RR");

        // Sets the movement direction of motors; different because of mirroring
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Tells the motors to lock/brake position to prevent stray movement when motor off
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
}
