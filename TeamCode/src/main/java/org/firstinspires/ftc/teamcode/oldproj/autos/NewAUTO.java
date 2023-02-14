package org.firstinspires.ftc.teamcode.oldproj.autos;

import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.TrajectoryBot;

import java.util.ArrayList;

@Autonomous(name="NewAUTO", group="PursuitBot")
public class NewAUTO extends LinearOpMode {

    public RamseteController controller;
    public MecanumDriveKinematics kinematics;
    public TrajectoryBot robot;

    public DcMotorEx FL, FR, BL, BR;

    //public VisionDevice vision;
    //public LinearSlide linearSlide;
    public int dropOffset = 80;

    public double maxVelocity = 2.0;
    public double maxAcceleration = 2.0;

    public boolean moveToNext;

    //auto
    @Override
    public void runOpMode() {

        robot = new TrajectoryBot(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 1.5;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2.5;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 1;

        controller = new RamseteController(2.0, 0.7);
        kinematics = new MecanumDriveKinematics(
                new Translation2d(-1 * robot.inchesToMeters(15.5/2), robot.inchesToMeters(8.5)),
                new Translation2d(robot.inchesToMeters(15.5/2), robot.inchesToMeters(8.5)),
                new Translation2d(-1 * robot.inchesToMeters(15.5/2), -1 * robot.inchesToMeters(8.5)),
                new Translation2d(robot.inchesToMeters(15.5/2), -1 * robot.inchesToMeters(8.5))
        );

        //vision = new VisionDevice(telemetry, hardwareMap);
        //vision.init();

        //linearSlide = new LinearSlide(telemetry, hardwareMap);
        sleep(1000);

        waitForStart();

        Trajectory t = createTraj(
                new Pose2d(robot.inchesToMeters(robot.xDim.toCell(2)), robot.inchesToMeters(robot.yDim.toCell(0)), new Rotation2d(Math.toRadians(90))),
                new Translation2d[]{new Translation2d(robot.inchesToMeters(robot.xDim.toCell(2) / 4), robot.inchesToMeters(robot.yDim.toCell(0))),
                        new Translation2d(robot.inchesToMeters(robot.xDim.toCell(2) / 4), robot.inchesToMeters(robot.yDim.toCell(0)))}, false);

        double seconds = t.getTotalTimeSeconds();
        ElapsedTime timer = new ElapsedTime();

        while(timer.seconds() < seconds)
        {
            robot.odometry.update();
            driveTraj(t, timer.seconds(), 0.05);
        }

        robot.drive.stop();
    }

    public Trajectory createTraj(Pose2d pose, Translation2d[] interiorPoints, boolean reversed) {
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        for (int i = 0; i < interiorPoints.length; i++) {
            interiorWaypoints.add(interiorPoints[i]);
        }
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(reversed);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                robot.odometry.getPose(), interiorWaypoints, pose, config);
        return trajectory;
    }

    public void driveTraj(Trajectory trajectory, double seconds, double inc) {
        Trajectory.State goal = trajectory.sample(seconds + inc);
        ChassisSpeeds adjustedSpeeds = controller.calculate(robot.odometry.getPose(), goal);
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);

        double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
        double frontRight = wheelSpeeds.frontRightMetersPerSecond;
        double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
        double backRight = wheelSpeeds.rearRightMetersPerSecond;

        robot.motorFL.setVelocity(frontLeft);
        robot.motorFR.setVelocity(frontRight);
        robot.motorBL.setVelocity(backLeft);
        robot.motorBR.setVelocity(backRight);

        /*double outputFL = robot.pidf.calculate(robot.motorFL.getCurrentPosition(), robot.velocityToTicks(frontLeft));
        double outputFR = robot.pidf.calculate(robot.motorFR.getCurrentPosition(), robot.velocityToTicks(frontRight));
        double outputBL = robot.pidf.calculate(robot.motorBL.getCurrentPosition(), robot.velocityToTicks(backLeft));
        double outputBR = robot.pidf.calculate(robot.motorBR.getCurrentPosition(), robot.velocityToTicks(backRight));*/

        /*PController controllerFL = new PController(robot.pidf.getP());
        PController controllerFR = new PController(robot.pidf.getP());
        PController controllerBL = new PController(robot.pidf.getP());
        PController controllerBR = new PController(robot.pidf.getP());

        controllerFL.setTolerance(5, 10);
        controllerFR.setTolerance(5, 10);
        controllerBL.setTolerance(5, 10);
        controllerBR.setTolerance(5, 10);

        PController[] controllers = new PController[]{controllerFL, controllerFR, controllerBL, controllerBR};
        DcMotor[] motors = new DcMotor[]{robot.motorFL, robot.motorFR, robot.motorBL, robot.motorBR};

        controllerFL.setSetPoint(robot.velocityToTicks(frontLeft));
        controllerFR.setSetPoint(robot.velocityToTicks(frontRight));
        controllerBL.setSetPoint(robot.velocityToTicks(backLeft));
        controllerBR.setSetPoint(robot.velocityToTicks(backRight));

        for (int i = 0; i < controllers.length; i++) {
            double output = controllers[i].calculate(
                    motors[i].getCurrentPosition()  // the measured value
            );
            DcMotorEx exM = (DcMotorEx)motors[i];
            exM.setVelocity(output);
        }*/
    }
}