package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;

// Rohan's teleop to test claw for cones
@TeleOp(name="ClawLiftTele", group="ClawLiftBot")
public class ClawLiftTele extends OpMode {

    public double armSpeed = 0.75;

    public double maxWheelSpeed = 0.75;
    public double turnSpeed = 0.75;
    public double linearSpeed = 1;

    // motor declaration
    public ServoEx clawL;
    public ServoEx clawR;
    public Motor armB;
    public Motor armF;

    public Motor left;
    public Motor right;

    // input system reference
    GamepadSystem input;

    // called on program initialization
    @Override
    public void init() {

        // initialize hardware devices
        clawL = new SimpleServo(hardwareMap, "clawL", 0, 180);
        clawR = new SimpleServo(hardwareMap, "clawR", 0, 180);
        armF = new Motor(hardwareMap, "armF");
        armB = new Motor(hardwareMap, "armB");

        armF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        left = new Motor(hardwareMap, "left");
        right = new Motor(hardwareMap, "right");

        input = new GamepadSystem(this);
    }

    // called repeatedly during program
    @Override
    public void loop() {

        double leftSpeed =
                input.gamepad1.getLeftY() * linearSpeed -
                input.gamepad1.getRightY() * turnSpeed;

        double rightSpeed =
                input.gamepad1.getLeftY() * linearSpeed +
                input.gamepad1.getRightY() * turnSpeed;

        leftSpeed = Math.max(Math.min(leftSpeed, 1), -1);
        rightSpeed = Math.max(Math.min(rightSpeed, 1), -1);

        leftSpeed = leftSpeed * Math.abs(leftSpeed);
        rightSpeed = rightSpeed * Math.abs(rightSpeed);

        left.set(leftSpeed * maxWheelSpeed);
        right.set(-rightSpeed * maxWheelSpeed);

        armF.set(input.gamepad2.getLeftY() * armSpeed);
        armB.set(input.gamepad2.getRightY() * armSpeed);

        //full close=1
        if (gamepad2.dpad_up) {
            clawL.setPosition(0.5);
            clawR.setPosition(0);
        }
        //full open=0
        if (gamepad2.dpad_down) {
            clawR.setPosition(0);
            clawR.setPosition(0.5);
        }

    }
}
