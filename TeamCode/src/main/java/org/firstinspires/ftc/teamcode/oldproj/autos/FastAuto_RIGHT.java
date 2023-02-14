package org.firstinspires.ftc.teamcode.oldproj.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.oldproj.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.oldproj.hardware.VisionDevice;

@Autonomous(name="FastAuto_RIGHT", group="PursuitBot")
public class FastAuto_RIGHT extends LinearOpMode {

    public double poleCellDiff;
    public PursuitBotTesting robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    ColorRangeSensor sensorColor;
    public int dropOffset = 225;
    public ColorSensor sensor;

    public boolean moveToNext;

    public Pose2d high;
    public Pose2d stack;
    public Pose2d midCell;
    public Pose2d midPoint;
    public Pose2d midPoint2;
    public Pose2d midCell2;
    public Pose2d beforePole;
    public Pose2d beforeRot;

    //auto
    @Override
    public void runOpMode() {
        robot = new PursuitBotTesting(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 0.125;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2;
        sensor = hardwareMap.colorSensor.get("sensor");
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 4;

        poleCellDiff = robot.xDim.toPole(1) - robot.xDim.toCell(1) - 1.5;
        high = new Pose2d(robot.xDim.toCell(2) - poleCellDiff, robot.yDim.toPole(3) - 1, Rotation2d.fromDegrees(180));
        beforePole = new Pose2d(robot.xDim.toCell(2), robot.yDim.toPole(3) - 1, Rotation2d.fromDegrees(180));
        beforeRot = new Pose2d(robot.xDim.toCell(2), robot.yDim.toPole(3) - 1, Rotation2d.fromDegrees(90));
        stack = new Pose2d(robot.xDim.toCell(2) + 5, robot.yDim.toCell(5) - 3.5, Rotation2d.fromDegrees(80));
        midPoint = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), Rotation2d.fromDegrees(180));
        midCell = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), Rotation2d.fromDegrees(90));

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
        sleep(300);

        // CONE GRABBED
        robot.setConstants(0.3, 0.19, 10, 4);
        robot.reachPointSlide(new Pose2d(robot.xDim.toPole(0), robot.yDim.toPole(3) - 1, new Rotation2d()), telemetry, this, linearSlide, linearSlide.low, false);
        linearSlide.goToFull(linearSlide.low + 75, telemetry, this);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);
        robot.setConstants(0.3, 0.19, 10, 4);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(0) , robot.yDim.toCell(4), new Rotation2d()), telemetry, this, linearSlide, linearSlide.low, true);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(0) , robot.yDim.toCell(4), Rotation2d.fromDegrees(90)), telemetry, this, linearSlide, linearSlide.low, true);
        robot.setConstants(0.8, 0.19, 24, 4);
        robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2) , robot.yDim.toCell(4), Rotation2d.fromDegrees(80)), telemetry, this, linearSlide, linearSlide.low, true);

        Cycle(0, false, result);
        Cycle(1, false, result);
        Cycle(2, false, result);
        Cycle(3, false, result);
        Cycle(4, true, result);
    }

    public void Cycle(int i, boolean lastCycle, int result)
    {
        robot.setConstants(0.8, 0.19, 24, 4);
        robot.reachPointSlide(stack, telemetry, this, linearSlide, linearSlide.stacks[i], false);
        robot.drive.stop();
        linearSlide.closeClaw();
        sleep(300);
        linearSlide.goTo(linearSlide.low, telemetry);
        sleep(300);
        robot.setConstants(0.8, 0.19, 24, 4);
        robot.reachPointSlide(beforeRot, telemetry, this, linearSlide, linearSlide.low, false);
        robot.reachPointSlide(beforePole, telemetry, this, linearSlide, linearSlide.med, false);
        robot.setConstants(0.3, 0.19, 10, 4);
        robot.reachPointSlide(high, telemetry, this, linearSlide, linearSlide.med, false);
        robot.drive.stop();
        linearSlide.goToFull(linearSlide.med + dropOffset, telemetry, this);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);
        robot.setConstants(0.3, 0.19, 10, 4);
        robot.reachPointSlide(beforePole, telemetry, this, linearSlide, linearSlide.low, false);
        robot.reachPointSlide(beforeRot, telemetry, this, linearSlide, linearSlide.low, false);
        if(!lastCycle) return;

        robot.setConstants(0.8, 0.19, 12, 6);

        if(result != 2) {
            robot.reachPointSlide(new Pose2d(beforePole.getX(), robot.yDim.toCell(result + 3), beforePole.getRotation()), telemetry, this, linearSlide, linearSlide.stacks[i + 1], false);
            robot.drive.stop();
        }
        else
        {
            robot.reachPointSlide(stack, telemetry, this, linearSlide, linearSlide.stacks[i + 1], false);
            robot.drive.stop();
        }
    }
}

