package org.firstinspires.ftc.teamcode.teleops;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.GamepadSystem;

// Rohan's teleop to test claw for cones
@TeleOp(name="ClawLiftTele", group="ClawLiftBot")
public class ClawLiftTele extends OpMode {

    PursuitBot robot;

    public double armSpeed = 0.75;

    public double turnSpeed = 0.75;
    public double linearSpeed = 0.75;
    public ElapsedTime timer = new ElapsedTime();

    // motor declaration
    //public ServoEx clawL;
    //public ServoEx clawR;
    //public Motor armB;
    //public Motor armF;
    public Motor slide;

    // input system reference
    GamepadSystem input;

    // called on program initialization
    @Override
    public void init() {

        robot = new PursuitBot(telemetry, hardwareMap);

        // initialize hardware devices
        //clawL = new SimpleServo(hardwareMap, "clawL", 0, 180);
        //clawR = new SimpleServo(hardwareMap, "clawR", 0, 180);

        slide = new Motor(hardwareMap, "slide");

        //armF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //armB.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //slide.setZeroPowerBehavior(null);

        input = new GamepadSystem(this);
    }

    // called repeatedly during program
    @Override
    public void loop() {

        robot.drive.driveRobotCentric(
                input.gamepad1.getLeftY() * linearSpeed,
                input.gamepad1.getLeftX() * linearSpeed,
                input.gamepad1.getRightX() * turnSpeed);

        if(gamepad2.y)
        {
            timer.reset();

            Up();
            Down();

            timer.reset();
            telemetry.addData("seconds", timer.seconds());
        }
        //armF.set(input.gamepad2.getRightY() * armSpeed);
        //armB.set(input.gamepad2.getLeftY() * armSpeed);
        slide.set(input.gamepad2.getRightY() * armSpeed);
        telemetry.addData("speed", input.gamepad2.getRightY() * armSpeed);
        //slide.set(input.gamepad2.getButton)
        //telemetry.addData("Linear Slide Position", slide.getDistance());
        //full close=1

        if (input.gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            //clawL.setPosition(1);
            //clawR.setPosition(0);
            telemetry.addData("servo", "close");
            telemetry.update();
        }
        //full open=0
        if (input.gamepad2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            //clawL.setPosition(0);
            //clawR.setPosition(1);
            telemetry.addData("servo", "open");
            telemetry.update();
        }

    }

    public void Up()
    {
        timer.reset();
        while(timer.seconds() <= 10)
        {
            slide.set(-0.75);
        }
    }

    public void Down()
    {
        timer.reset();
        while(timer.seconds() <= 10)
        {
            slide.set(-0.75);
        }
    }
}
