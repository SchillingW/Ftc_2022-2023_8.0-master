package org.firstinspires.ftc.teamcode.newproj.prototype;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DistanceDemo",group="LeaguePrep")
public class DistanceDemo extends OpMode {

    public DistanceSensor distance;

    @Override
    public void init() {

        distance = hardwareMap.get(DistanceSensor.class, "sensor");
    }

    @Override
    public void loop() {

        telemetry.addData("distance", String.format("%.2f", distance.getDistance(DistanceUnit.INCH)));
    }
}
