package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Nav Routines Test", group = "Tests")
public class NavRountinesTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        go_forward(48,0,.3,false, false, 0);
        sleep(5000);
        go_forward(12,0,.3,false, false, 0);
    }
}
