package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="AutoTest", group="PursuitBot")
public class AutoTest extends LinearOpMode {

    public PursuitBot robot;
    public VisionDevice vision;
    public ElapsedTime timer = new ElapsedTime();
    public Motor slide;
    public Servo claw;

    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 2;
        robot.yDim.cellcorner2botanchorPLACEMENT = 1;
        robot.xDim.cellPLACEMENT = 1;
        robot.yDim.cellPLACEMENT = 0;

        slide = new Motor(hardwareMap, "slide");
        claw = hardwareMap.servo.get("claw");

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();
        int result = 0;
        while (!isStarted()) result = vision.perform(1f / 3f);

        waitForStart();

        // START MOVEMENT

        if (opModeIsActive()) slide.set(0);
        if (opModeIsActive()) claw.setPosition(0);
        if (opModeIsActive()) sleep(2000);


        // CONE GRABBED

        robot.reachPoint(new Pose2d(robot.xDim.toCell(0) + 1, robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        if (opModeIsActive()) Up();
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(2), new Rotation2d()), telemetry, this);

        // AT DROP CONE LOCATION

        if (opModeIsActive()) sleep(2000);
        Down();
        //if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) claw.setPosition(0.5);
        //if (opModeIsActive()) slide.set(1);


        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(0) + 1, robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(0) + 1, robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCone(1), robot.yDim.toCone(1), new Rotation2d()), telemetry, this);

        // AT CONE GRAB LOCATION

        if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) claw.setPosition(0);
        if (opModeIsActive()) sleep(2000);

        // CONE GRABBED

        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        if (opModeIsActive()) Up();
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(2), new Rotation2d()), telemetry, this);

        // AT DROP LOCATION

        if (opModeIsActive()) sleep(2000);
        Down();

        //if (opModeIsActive()) sleep(500);
        //if (opModeIsActive()) slide.set(1);
        if (opModeIsActive()) claw.setPosition(0.5);
        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(result), new Rotation2d()), telemetry, this);
    }

    public void Up()
    {
        timer.reset();

        while(opModeIsActive() && timer.seconds() <=0.5)
        {
            slide.set(-1);
        }
    }

    public void Down()
    {
        timer.reset();

        while(opModeIsActive() && timer.seconds() <= 0.5)
        {
            slide.set(1);
        }
    }
}

