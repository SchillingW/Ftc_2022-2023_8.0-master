package org.firstinspires.ftc.teamcode.teleops;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Rohan's teleop to test claw for cones
@TeleOp(name="ClawLiftTele", group="ClawLiftBot")
public class ClawLiftTele extends OpMode {

    // motor declaration
    public ServoEx clawServo;
    public Motor primaryArm;
    public Motor secondaryArm;

    // called on program initialization
    @Override
    public void init() {

        // initialize hardware devices
        primaryArm = new Motor(hardwareMap, "primaryArm");
        secondaryArm = new Motor(hardwareMap, "secondaryArm");
        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 180);
    }

    // called repeatedly during program
    @Override
    public void loop() {

        primaryArm.set(gamepad1.left_stick_y);
        secondaryArm.set(gamepad1.right_stick_y);

        if (gamepad1.dpad_up) clawServo.setPosition(1);
        if (gamepad1.dpad_down) clawServo.setPosition(0);

    }
}
