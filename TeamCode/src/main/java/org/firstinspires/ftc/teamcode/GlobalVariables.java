package org.firstinspires.ftc.teamcode;

public class GlobalVariables{

    public static final boolean logging_10435 = true;

    public static final double MINERAL_BOX_ADJUSTMENT = .01; // When new box servo is installed, should be the only variable that needs to be changed.  Accounts for difference where the servo spline grooves line up.
                                                             // Servo should be set to defaults for full range of motion (no right/left programming)
                                                             // It should be zero if box is perfectly square with the ground while servo tester is at middle pos.  It should be positive if the front edge of the box is tipped up
    public static final double MINERAL_BOX_GROUND_LEVEL = .50 - MINERAL_BOX_ADJUSTMENT; //the pos where the mineral box is square with the ground.  Everything based off of this position
    public static final double MINERAL_BOX_FULL_DROP = MINERAL_BOX_GROUND_LEVEL + .35;   // highest drop angle possible without tumbling the cubes
    public static final double MINERAL_BOX_HALF_DROP = MINERAL_BOX_GROUND_LEVEL + .22;   // enough angle for the ball to roll out, but the cube stays
    public static final double MINERAL_BOX_INTAKE_POS = MINERAL_BOX_GROUND_LEVEL + .02;  // the angle the box is at when we are intaking minerals
    public static final double MINERAL_BOX_BEGINLIFT_POS = MINERAL_BOX_GROUND_LEVEL + .35;  // the angle the box turns to when B is pressed
    public static final double MINERAL_BOX_LIFTED_POS = MINERAL_BOX_GROUND_LEVEL - .13;  // the angle the box ends up when the mineral arm has turned fully to the drop position
    public static final double MINERAL_BOX_BEGINSCOOP_POS = MINERAL_BOX_GROUND_LEVEL - .27;  // the angle the box is at when lowering into the crater
    public static final double MINERAL_BOX_HALFDROP_TURN_TIME = (MINERAL_BOX_HALF_DROP - MINERAL_BOX_LIFTED_POS) * 1.0;  // the number of seconds it takes to spin the the box to the half drop level
    public static final double MINERAL_BOX_HALFDROP_PAUSE_TIME = .5;  // the number of seconds to pause before full drop

    public static final double MINERAL_INTAKE_FORWARD_SPEED = -.88;  // VEX motors don't take the full range, so .88 seems to be the max
    public static final double MINERAL_INTAKE_REVERSE_SPEED = .88;  // VEX motors don't take the full range, so .88 seems to be the max

    public static final int MAX_MIL1_TICKS = 1900; // was 471 with Core Hex motors

    public static final int LIFT_DROP_TICKS = 1150;

}
