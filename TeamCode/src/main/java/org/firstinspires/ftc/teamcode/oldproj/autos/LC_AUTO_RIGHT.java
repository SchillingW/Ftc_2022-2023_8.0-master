package org.firstinspires.ftc.teamcode.oldproj.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.oldproj.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.oldproj.hardware.VisionDevice;

@Autonomous(name="LC_AUTO_RIGHT", group="PursuitBot")
public class LC_AUTO_RIGHT extends LinearOpMode {

    public double poleCellDiff;
    public PursuitBotTesting robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    ColorSensor sensorColor;
    public int dropOffset = 225;
    public ColorSensor sensor;

    public boolean moveToNext;

    public Pose2d high;
    public Pose2d stack;
    public Pose2d midCell;
    public Pose2d midPoint;
    public Pose2d midPoint2;
    public Pose2d midCell2;

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
        high = new Pose2d(robot.xDim.toCell(2) + 2.5, robot.yDim.toPole(3) - 0.425, new Rotation2d());
        stack = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 2.3, Rotation2d.fromDegrees(90));
        midPoint = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), new Rotation2d());
        midCell = new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), Rotation2d.fromDegrees(85));
        midPoint2 = new Pose2d(robot.xDim.toCell(2), robot.yDim.toPole(3), new Rotation2d());
        midCell2 = new Pose2d(robot.xDim.toCell(2), robot.yDim.toPole(3), Rotation2d.fromDegrees(90));

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
        //robot.reachPointSlide(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, linearSlide, linearSlide.driveHeight, false);
        robot.setConstants(0.9, 0.6, 16, 4);
        robot.reachPointSlideNextPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(4), new Rotation2d()), new Pose2d(robot.xDim.toCell(1.8), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, linearSlide, linearSlide.high, false);
        robot.setConstants(0.6, 0.6, 16, 4);
        robot.reachPointSlideNextPoint(new Pose2d(robot.xDim.toCell(1.8), robot.yDim.toCell(4), new Rotation2d()), high, telemetry, this, linearSlide, linearSlide.high, false);
        robot.setConstants(0.6, 0.25, 16, 2);
        robot.reachPointSlide(high, telemetry, this, linearSlide, linearSlide.high, false);
        robot.drive.stop();
        linearSlide.goToFull(linearSlide.high + dropOffset, telemetry, this);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);


        Cycle(0, false, result);
        Cycle(1, false, result);
        Cycle(2, false, result);
        Cycle(3, true, result);
    }

    public void Cycle(int i, boolean lastCycle, int result)
    {
        robot.setConstants(0.85, 0.55, 12, 6);
        //robot.reachPointSlideNextPoint(midCell, new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 1.3, Rotation2d.fromDegrees(90)), telemetry, this, linearSlide, linearSlide.med, false);
        robot.reachPointSlideNextPoint(midPoint, midCell, telemetry, this, linearSlide, linearSlide.med, false);
        robot.setConstants(0.55, 0.4, 16, 2);
        //robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 1.3, Rotation2d.fromDegrees(90 - (i * 0.65))), telemetry, this, linearSlide, linearSlide.stacks[i], false);
        robot.reachPointSlide(midCell, telemetry, this, linearSlide, linearSlide.stacks[i], false);
        robot.setConstants(0.4, 0.19, 16, 4);
        robot.reachPointSlide(stack, telemetry, this, linearSlide, linearSlide.stacks[i], false);
        robot.drive.stop();
        linearSlide.closeClaw();
        sleep(300);
        linearSlide.goTo(linearSlide.high, telemetry);
        if(!lastCycle) sleep(150);
        else sleep(450);
        robot.setConstants(0.69, 0.3, 16, 4);
        robot.reachPointSlide(midCell, telemetry, this, linearSlide, linearSlide.high, false);
        robot.setConstants(0.4, 0.25, 16, 2);
        robot.reachPointSlide(midPoint, telemetry, this, linearSlide, linearSlide.high, false);
        robot.setConstants(1, 0.3, 16, 4);
        //robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2) + 3.2 + i * 0.1, robot.yDim.toPole(3), Rotation2d.fromDegrees(i * 1.25)), telemetry, this, linearSlide, linearSlide.high, false);
        robot.reachPointSlide(high, telemetry, this, linearSlide, linearSlide.high, false);
        robot.drive.stop();
        linearSlide.goToFull(linearSlide.high + dropOffset, telemetry, this);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(200);
        if(!lastCycle) return;

        //PARK
        robot.setConstants(1, 0.3, 12, 2);
        if(result != 2) robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2) - 1.5, robot.yDim.toCell(result + 3), new Rotation2d()), telemetry, this, linearSlide, linearSlide.low, false);
        else
        {
            robot.setConstants(1, 0.45, 12, 6);
            robot.reachPointSlideNextPoint(midCell, new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 1.55, Rotation2d.fromDegrees(90)), telemetry, this, linearSlide, linearSlide.med, false);
            robot.setConstants(0.45, 0.19, 16, 2);
            robot.reachPointSlide(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 1.55, Rotation2d.fromDegrees(90 - ((i + 1) * 0.65))), telemetry, this, linearSlide, linearSlide.stacks[i], false);
            robot.drive.stop();
        }
    }
}

