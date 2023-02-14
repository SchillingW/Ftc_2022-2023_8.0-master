package org.firstinspires.ftc.teamcode.oldproj.teleops;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBotTesting;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.oldproj.hardware.GamepadSystem;

import java.io.LineNumberReader;

@TeleOp(name="SensorTestEesh", group="ClawLiftBot")
public class SensorTestEesh extends OpMode {

    ColorRangeSensor sensorColor;
    LinearSlide slide;

    // called on program initialization
    @Override
    public void init() {
        sensorColor = (ColorRangeSensor) hardwareMap.colorSensor.get("sensor");
        slide = new LinearSlide(telemetry, hardwareMap);
    }

    // called repeatedly during program
    @Override
    public void loop() {
        telemetry.addData("Slide Pos", slide.getCurrentPos());
        telemetry.addData("Sensor Color Blue", sensorColor.blue());
        telemetry.addData("Sensor Color Green", sensorColor.green());
        telemetry.addData("Sensor Color Red", sensorColor.red());
        telemetry.addData("Sensor Distance Inches", sensorColor.getDistance(DistanceUnit.INCH));

    }
}
