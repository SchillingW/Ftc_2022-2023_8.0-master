package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.botconfigs.TrajectoryBot;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous(name="NewAUTO", group="PursuitBot")
public class NewAUTO extends LinearOpMode {

    public RamseteController controller;
    public DifferentialDriveKinematics kinematics;
    public TrajectoryBot robot;
    //public VisionDevice vision;
    //public LinearSlide linearSlide;
    public int dropOffset = 80;

    public double maxVelocity = 0.5;
    public double maxAcceleration = 0.5;

    public boolean moveToNext;

    //auto
    @Override
    public void runOpMode() {

        robot = new TrajectoryBot(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 1.5;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2.5;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 1;

        controller = new RamseteController(78.7401574, 27.5590551);
        kinematics = new DifferentialDriveKinematics(0.3683);

        //vision = new VisionDevice(telemetry, hardwareMap);
        //vision.init();

        //linearSlide = new LinearSlide(telemetry, hardwareMap);
        sleep(1000);

        waitForStart();

        Trajectory t = createTraj(
                new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0), new Rotation2d(Math.toRadians(90))),
                new Translation2d[]{new Translation2d(robot.xDim.toCell(2) / 4, robot.yDim.toCell(0)),
                        new Translation2d(robot.xDim.toCell(2) / 4, robot.yDim.toCell(0))}, false);
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

    public void driveTraj(Trajectory trajectory, double seconds) {
        Trajectory.State goal = trajectory.sample(seconds);
        ChassisSpeeds adjustedSpeeds = controller.calculate(robot.odometry.getPose(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);

        double left = wheelSpeeds.leftMetersPerSecond;
        double right = wheelSpeeds.rightMetersPerSecond;

        double outputFL = robot.pidf.calculate(robot.motorFL.getCurrentPosition(), robot.velocityToTicks(left));
        double outputFR = robot.pidf.calculate(robot.motorFR.getCurrentPosition(), robot.velocityToTicks(right));
        double outputBL = robot.pidf.calculate(robot.motorBL.getCurrentPosition(), robot.velocityToTicks(left));
        double outputBR = robot.pidf.calculate(robot.motorBR.getCurrentPosition(), robot.velocityToTicks(right));

        PController controllerFL = new PController(robot.pidf.getP());
        PController controllerFR = new PController(robot.pidf.getP());
        PController controllerBL = new PController(robot.pidf.getP());
        PController controllerBR = new PController(robot.pidf.getP());

        PController[] controllers = new PController[]{controllerFL, controllerFR, controllerBL, controllerBR};
        Motor[] motors = new Motor[]{robot.motorFL, robot.motorFR, robot.motorBL, robot.motorBR};

        controllerFL.setSetPoint(robot.velocityToTicks(left));
        controllerFR.setSetPoint(robot.velocityToTicks(right));
        controllerBL.setSetPoint(robot.velocityToTicks(left));
        controllerBR.setSetPoint(robot.velocityToTicks(right));


        while (!controllerFL.atSetPoint() || !controllerFR.atSetPoint() || !controllerBL.atSetPoint() || !controllerBR.atSetPoint()) {
            for (int i = 0; i < controllers.length; i++) {
                while (!controllers[i].atSetPoint()) {
                    double output = controllers[i].calculate(
                            motors[i].getCurrentPosition()  // the measured value
                    );
                 
                }
            }
        }
    }
}