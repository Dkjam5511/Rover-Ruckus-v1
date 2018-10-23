package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Nav_Routines;

@Autonomous (name = "Deploy Marker Test", group = "Tests")
public class DeployMarkerTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        deploymarker();
    }
}
