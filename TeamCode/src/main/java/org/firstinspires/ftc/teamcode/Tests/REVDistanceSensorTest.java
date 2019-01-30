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
    Rev2mDistanceSensor rightdistancesensor;
    Rev2mDistanceSensor backdistancesensor;

    @Override
    public void init() {
        leftdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "lds");
        rightdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "rds");
        backdistancesensor = hardwareMap.get(Rev2mDistanceSensor.class, "bds");
    }
    @Override
    public void loop() {
        telemetry.addData("Distance Left: ", leftdistancesensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance Right: ", rightdistancesensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Distance Back: ", backdistancesensor.getDistance(DistanceUnit.INCH));



    }
}
