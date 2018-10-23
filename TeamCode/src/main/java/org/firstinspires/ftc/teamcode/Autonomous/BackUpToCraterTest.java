package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Nav_Routines;

@Autonomous(name = "Backup To Crater", group = "Tests")
public class BackUpToCraterTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        wallfollow(84, 0, -.35, 6, false,true);
    }
}
