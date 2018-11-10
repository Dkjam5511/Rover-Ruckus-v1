package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Nav Routines Test", group = "Tests")
public class NavRountinesTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
       go_sideways(90, 0 ,.3,1000, false);
       sleep(5000);
       go_sideways(270, 0 ,.3, 1000, false);
       sleep(5000);
       go_sideways(45,0,.3,1000, false);
       sleep(5000);
       go_sideways(225, 0, .3, 1000, false);
    }
}
