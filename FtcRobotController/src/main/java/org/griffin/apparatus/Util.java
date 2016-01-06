package org.griffin.apparatus;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Ultimate on 1/4/2016.
 *
 * Utility functions that are not member functions
 */
public class Util {

    private static float gainHigh = 0.3f;
    private static float gainLow = 0.9f;
    private static float defaultDeadband = 0.009f;

    /// give an acceleration profile to the throttle according to the equation
    ///
    /// y=G*((x-D)/(1-D))^3+(1-G)*(x-D)/(1-D); Where X is in value, D is Deadband and G is Gain (0..1)
    /// plug this equation into https://www.desmos.com/calculator an vary the values for
    ///
    ///  gain (0.0 <= G <= 1.0) and
    ///  deadband (0.0 <= D <= 1.0)
    ///
    ///  notice that the curve always goes through 1.0. The paramters allow us to make
    ///  the more precise (at low throttle positions, more input is required)
    ///
    ///  gainHigh is more linear response than gainLow
    ///
    public static float throttle(Gamepad gamepad, float rawThrottle, float deadband)
    {
        float gain = gainHigh;
        if (gamepad.right_bumper)
        { gain = gainLow; }

        /// save the sign of the joystick and use the absolute value -- this function
        /// only works for values >0
        ///
        float sign = Math.signum(rawThrottle);
        rawThrottle = Math.abs(rawThrottle);

        /// cube term without calling the Math.pow function
        ///
        float term = (rawThrottle-deadband)/(1.0f-deadband);
        float temp = term;
        temp *= temp;
        temp *= temp;

        float ret = gain * (temp + (1.0f - gain) * term);
        return Math.max(ret, 0.0f) * sign;
    }

    public static float throttle(Gamepad gamepad, float rawThrottle)
    {
        return throttle(gamepad, rawThrottle,defaultDeadband);
    }
}
