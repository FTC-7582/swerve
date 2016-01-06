package org.usfirst.ftc.griffin.apparatus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.IFunc;

import java.util.ArrayList;
import java.util.List;

import static org.usfirst.ftc.griffin.apparatus.Util.throttle;

/**
 * Created by Ultimate on 1/5/2016.
 *
 * Winch is comprised of two items - a motor and a cam on the servo
 */
public class Winch extends OpMode {
    public Servo winchServo;
    private double winchPosition = winchPositionLower;
    static double winchPositionLower = 0.0;
    static double winchStep = 1.0 / 180.0; // step in 1 degree increments

    public DcMotor winchMotor;
    private double winchMotorPowerCommanded;

    private String winchMotorName;
    private String winchServoName;

    private DcMotor.Direction winchMotorDirection;
    private String status;

    private TelemetryDashboardAndLog dashboard;
    private boolean logging;

    public Winch(String motorName, DcMotor.Direction motorDirection, String servoName, TelemetryDashboardAndLog dashboard, boolean logging)
    {
        this.winchMotorName = motorName;
        this.winchMotorDirection = motorDirection;

        this.winchServoName = servoName;

        this.dashboard = dashboard;
        this.logging = logging;
    }

    @Override
    public void init() {
        status = "";

        try {
            winchMotor = hardwareMap.dcMotor.get(winchMotorName);
            status += "W";
        } catch (Exception E) {
            winchMotor = null;
            status += "-";
        }

        try {
            winchServo = hardwareMap.servo.get(winchServoName);
            status += "w";
        } catch (Exception E) {
            winchServo = null;
            status += "-";
        }

        if (logging) {
            dashboard.addLine(dashboard.item("Winch servo/cmd/act ", new IFunc<Object>() {
                @Override
                public String value() {
                    String winchMotorPowerActualAsString = "";
                    String winchMotorPowerCommandedAsString = "";
                    String winchServoAsString = "";

                    if (winchServo != null) {
                        winchServoAsString = String.format("%.2f", winchServo.getPosition());
                    }
                    if (winchMotor != null) {
                        winchMotorPowerActualAsString = String.format("%.2f", winchMotor.getPower());
                        winchMotorPowerCommandedAsString = String.format("%.2f", winchMotorPowerCommanded);
                    }

                    return winchServoAsString + "/" + winchMotorPowerCommandedAsString + "/" + winchMotorPowerActualAsString;
                }
            }));
        }
    }

    @Override
    public void init_loop() {
        if (winchMotor == null)
        { return; }

        winchMotor.setDirection(winchMotorDirection);
    }

    @Override
    public void loop() {
        /// gamepad2.right_stick_y is used to deploy and retract winch (wincher-upper)
        ///
        float stickValue = Range.clip(-gamepad2.right_stick_y, -1.0f, 1.0f);
        winchMotorPowerCommanded = throttle(gamepad2, stickValue);

        if (winchMotor != null)
        { winchMotor.setPower(winchMotorPowerCommanded); }

        /// winchPosition uses gamepad2 dpad
        /// down is to lower the winch position
        /// up is to raise the winch position
        ///
        if (gamepad2.dpad_down) {
            winchPosition -= winchStep;
        } else if (gamepad2.dpad_up) {
            winchPosition += winchStep;
        }

        // clip the position values so that they never exceed 0..1
        winchPosition = Range.clip(winchPosition, 0, 1);

        if (winchServo != null)
        { winchServo.setPosition(winchPosition); }
    }

    // returns the list of active servos
    public List<Servo> activeServos()
    {
        List<Servo> ret = new ArrayList<Servo>();
        if (winchServo != null)
        { ret.add(winchServo); }

        return ret;
    }

    public String getStatus()
    {
        return status;
    }
}
