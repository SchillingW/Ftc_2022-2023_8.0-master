package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="LowAutoRight", group="PursuitBot")
public class LowAutoRight extends LinearOpMode {

    public double poleCellDiff;
    public PursuitBotTesting robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    ColorSensor sensorColor;
    public int dropOffset = 40;
    public ColorSensor sensor;

    public boolean moveToNext;

    //auto
    @Override
    public void runOpMode() {
        robot = new PursuitBotTesting(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 0.125;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2;
        sensor = hardwareMap.colorSensor.get("sensor");
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 4;
        robot.maxSpeed = 0.8;

        poleCellDiff = robot.xDim.toPole(1) - robot.xDim.toCell(1) + 1.25;

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();

        linearSlide = new LinearSlide(telemetry, hardwareMap);
        sleep(1000);
        int result = 0;
        while (!isStarted()) {
            int next = vision.perform(1f / 3f);
            if (next != -1) result = next;
            telemetry.addData("current result", result);
            telemetry.update();
        }
        // START MOVEMENT
        waitForStart();
        if (opModeIsActive()) linearSlide.closeClaw();
        sleep(200);

        // CONE GRABBED
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(0.5), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, linearSlide, linearSlide.driveHeight, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(1.5) , robot.yDim.toCell(4) + poleCellDiff, Rotation2d.fromDegrees(90)), telemetry, this, linearSlide, linearSlide.low, true);
        if (opModeIsActive()) linearSlide.openClaw();

        Cycle(0);
        Cycle(1);
        Cycle(2);
        Cycle(3);
        Cycle(4);

        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4) + poleCellDiff, Rotation2d.fromDegrees(90)), telemetry, this, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(result + 3), Rotation2d.fromDegrees(90)), telemetry, this, linearSlide, linearSlide.ground, true);
    }

    public void Cycle(int i)
    {
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4) + poleCellDiff, Rotation2d.fromDegrees(90)), telemetry, this, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 0.75, Rotation2d.fromDegrees(90)), telemetry, this, linearSlide, linearSlide.stacks[i], true);
        linearSlide.closeClaw();
        sleep(200);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4) + poleCellDiff, Rotation2d.fromDegrees(90)), telemetry, this, linearSlide, linearSlide.low - 50, false);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(1.5) , robot.yDim.toCell(4) + poleCellDiff, Rotation2d.fromDegrees(90)), telemetry, this, linearSlide, linearSlide.low, true);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);
    }
}

