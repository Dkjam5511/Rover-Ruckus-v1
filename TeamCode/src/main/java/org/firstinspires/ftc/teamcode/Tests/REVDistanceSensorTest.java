package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "REV Distance Sensor Test", group = "Tests")
public class REVDistanceSensorTest extends OpMode {

    Rev2mDistanceSensor leftdistancesensor;
    ModernRoboticsI2cRangeSensor rightdistancesensor;
    ColorSensor cs2;
    DistanceSensor rightcds2;

    @Override
    public void init() {
        leftdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "lds");
        rightdistancesensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rds");
        cs2 = hardwareMap.get(ColorSensor.class, "mcs2");
        rightcds2 = hardwareMap.get(DistanceSensor.class, "mcs2");
    }
    @Override
    public void loop() {

        telemetry.addData("DistanceLeft", leftdistancesensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Raw Ultrasonic", rightdistancesensor.rawUltrasonic());
        telemetry.addData("Raw Optical", rightdistancesensor.rawOptical());
        telemetry.addData("Distance", rightdistancesensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Color Distance", rightcds2.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right Alpha", cs2.alpha());
        telemetry.addData("Right Red", cs2.red());
        telemetry.addData("Right Blue", cs2.blue());
        telemetry.addData("Right Green", cs2.green());


    }
}
