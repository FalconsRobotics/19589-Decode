package old_do_not_touch.Constants;

import com.acmerobotics.dashboard.config.Config;

/// A constants class just for having all the "tuneable" values
/// related to the shooter in one place for simplicity.
@Config
public class ShooterConstants {
    public static double HIGH_RPM_RANGE = 4600; // in RPMs
    public static double LOW_RPM_RANGE = 4300; // in RPMS
    public static double FIRING_SPEED_LONG = 4400; // old 4500

    public static double FIRING_SPEED_SHORT = 2300;

    public static double MOTOR_RPMS = 28; // Taken from GoBilda, from a 6000RPM motor.

    // PID Values, tune as necessary
    public static double kP = 1;
    public static double kI = 1;
    public static double kD = 1;
    public static double kF = 1;
}
