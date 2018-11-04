/* find line test 2 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Find Line Test", group = "Tests")
public class FindLineTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        wallfollow(120, 0, .3, 16, false, false);
    }
}
