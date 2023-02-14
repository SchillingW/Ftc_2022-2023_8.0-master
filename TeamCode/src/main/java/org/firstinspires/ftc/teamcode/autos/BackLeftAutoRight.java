package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="RotationAutoRight", group="PursuitBot")
public class BackLeftAutoRight extends LinearOpMode {

    public PursuitBotTesting robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    ColorSensor sensorColor;
    public int dropOffset = 40;
    public ColorSensor sensor;
    RevBlinkinLedDriver lights;

    public boolean moveToNext;

    //auto
    @Override
    public void runOpMode() {

        robot = new PursuitBotTesting(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 2.375;
        robot.yDim.cellcorner2botanchorPLACEMENT = 0.375;
        sensor = hardwareMap.colorSensor.get("sensor");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 4;

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();

        linearSlide = new LinearSlide(telemetry, hardwareMap);
        sleep(1000);
        int result = 0;
        while (!isStarted()) {

            int next = vision.perform(1f / 3f);
            if (next != -1) result = next;
            if (next == 0) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            if (next == 1) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            if (next == 2) lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            telemetry.addData("current result", result);
            telemetry.update();
        }
        // START MOVEMENT
        waitForStart();
        if (opModeIsActive()) linearSlide.closeClaw();
        sleep(200);

        // CONE GRABBED
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.driveHeight, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.high, true);
        if (opModeIsActive()) linearSlide.openClaw();

        Cycle(0);
        Cycle(1);
        Cycle(2);

        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.low, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(result + 3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.ground, true);
    }

    public void Cycle(int i)
    {
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.low, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5), new Rotation2d()), telemetry, this, linearSlide, linearSlide.stacks[i], true);
        sleep(500);
        linearSlide.closeClaw();
        sleep(200);
        //linearSlide.goToFull(linearSlide.stackDriveHeight, telemetry, this);

        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(3), new Rotation2d()), telemetry, this,  false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.high, true);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);
    }
}

