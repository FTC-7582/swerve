package org.usfirst.ftc.griffin.apparatus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.IFunc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Ultimate on 1/4/2016.
 * The HT7592-HB quarter scale servo that appears to go 3.5 (or so) turns CW
 */
public class FunkyContinuous extends OpMode {
    /// I found this on the web. Don't know if it is true -JGM
    ///
    //    Given that the HiTechic Servo Controller allows setting of the PWM output from 750 to 2250 microseconds
    //    with a step resolution of 5.88 microseconds anything under servo[foo] = 42 puts the servo into the zombie state.
    //    The math is the same for the other end of the scale (212).
    //
    //    Anything in between those two values will fall somewhere within the 1260 degree rotation of the servo.
    //
    public Servo continuousServo;
    private double continuousServoMovement;
    private static double continuousRotateLeft = 43.0 / 256.0;
    private static double continuousMiddle = 0.5;
    private static double continuousRotateRight = 211.0 / 256.0;
    private static double continuousRotateStep = 0.1;

    private String continuousServoName;
    private String status;

    private TelemetryDashboardAndLog dashboard;
    private boolean logging;

    public FunkyContinuous(String servoName, TelemetryDashboardAndLog dashboard, boolean logging)
    {
        this.continuousServoName = servoName;
        this.dashboard = dashboard;
        this.logging = logging;
    }

    @Override
    public void init() {
        status = "";

        try {
            continuousServo = hardwareMap.servo.get(continuousServoName);
            status += "c";
        } catch (Exception E) {
            continuousServo = null;
            status += "-";
        }


        dashboard.addLine(dashboard.item("Continuous ", new IFunc<Object>() {
            @Override
            public String value() {
                String positionAsString = "";

                if (continuousServo != null)
                { positionAsString = String.format("%.2f", continuousServo.getPosition()); }

                return  positionAsString;
            }
        }));

    }

    @Override
    public void init_loop() {
        continuousServoMovement = continuousMiddle;
    }

    @Override
    public void loop() {
        /// Continuous servo
        /// X means set continuousServoPosition to go left
        /// Y means stop
        /// B means set continuousServoPosition to go right
        /// A means move the continuous servo positive 10% of travel (Experimental)
        /// A + Left Bumper means move the continuous servo negative 10% of travel (Experimental)
        ///
        if (gamepad1.x) {
            continuousServoMovement = continuousRotateLeft;
        } else if (gamepad1.y) {
            continuousServoMovement = continuousMiddle;
        } else if (gamepad1.b) {
            continuousServoMovement = continuousRotateRight;
        } else if (gamepad1.a) {
            if (gamepad1.left_bumper) {
                continuousServoMovement -= continuousRotateStep;
            } else {
                continuousServoMovement += continuousRotateStep;
            }
        }

        if (continuousServo != null)
        { continuousServo.setPosition(continuousServoMovement); }
    }

    // returns the list of active servos
    public List<Servo> activeServos()
    {
        List<Servo> ret = new ArrayList<Servo>();
        if (continuousServo != null)
        { ret.add(continuousServo); }

        return ret;
    }

    public String getStatus()
    {
        return status;
    }
}

