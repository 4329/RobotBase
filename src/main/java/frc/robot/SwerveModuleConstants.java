package frc.robot;

public final class SwerveModuleConstants
{
    public static final int CAN_TIMEOUT = 20;
    public static final int CURRENT_LIMIT_STALL = 40; //Current limit at 0 RPM
    public static final int CURRENT_LIMIT_FREE = 20;  //Current limit when free moving
    public static final int PERIODIC_FRAME_PERIOD = 5;

    // Order for all arrays: (BL, BR, FR, FL)
    public static final int[] TRANSLATION_PORT =
    { 1, 3, 5, 7 };
    public static final int[] ROTATION_PORT =
    { 2, 4, 6, 8 };
    public static final int[] POTENTIOMETER_PORT =
    { 0, 1, 2, 3 };
    public static final double[] OFFSET =
    { Configrun.get(90.0, "offset0"), Configrun.get(293.9, "offset1"), Configrun.get(33.9, "offset2"),
            Configrun.get(87.6, "offset3") };
}