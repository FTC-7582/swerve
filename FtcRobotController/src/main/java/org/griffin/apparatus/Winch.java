package org.griffin.apparatus;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.IFunc;

import java.util.ArrayList;
import java.util.List;

import static org.griffin.apparatus.Util.throttle;

/**
 * Created by Ultimate on 1/5/2016.
 *
 * Winch is comprised of two items - a motor and a cam on the servo
 */
public class Winch {
    public Servo heightServo;
    private double winchHeight = winchHeightLower;
    static double winchHeightLower = 0.0;
    static double winchHeightStep = 1.0 / 180.0; // step in 1 degree increments

    public Servo deployServo;
    static double deployServoUnwind = 0.0;
    static double deployServoStop = 0.5;
    static double deployServoWind = 1.0;

    public DcMotor spoolMotor;
    private double spoolMotorPowerCommanded;

    private String spoolMotorName;
    private String heightServoName;
    private String deployServoName;

    private DcMotor.Direction spoolMotorDirection;
    private String status;

    private TelemetryDashboardAndLog dashboard;
    private boolean logging;

    private OpMode opMode;

    public Winch(OpMode opMode, String motorName, DcMotor.Direction motorDirection, String heightServoName, String deployServoName, TelemetryDashboardAndLog dashboard, boolean logging)
    {
        this.opMode = opMode;

        this.spoolMotorName = motorName;
        this.spoolMotorDirection = motorDirection;

        this.heightServoName = heightServoName;
        this.deployServoName = deployServoName;

        this.dashboard = dashboard;
        this.logging = logging;
    }

    public void init() {
        status = "";

        try {
            spoolMotor = opMode.hardwareMap.dcMotor.get(spoolMotorName);
            status += "W";
        } catch (Exception E) {
            DbgLog.msg("*** Exc Winch." + spoolMotorName + " " + E.getMessage());
            spoolMotor = null;
            status += "-";
        }

        try {
            heightServo = opMode.hardwareMap.servo.get(heightServoName);
            status += "w";
        } catch (Exception E) {
            DbgLog.msg("*** Exc Winch." + heightServoName + " " + E.getMessage());
            heightServo = null;
            status += "-";
        }

        try {
            deployServo = opMode.hardwareMap.servo.get(deployServoName);
            status += "d";
        } catch (Exception E) {
            DbgLog.msg("*** Exc Winch." + deployServoName + " " + E.getMessage());
            heightServo = null;
            status += "-";
        }

        if (logging) {
            dashboard.addLine(dashboard.item("Winch servo/cmd/act ", new IFunc<Object>() {
                @Override
                public String value() {
                    String spoolMotorPowerActualAsString = "";
                    String spoolMotorPowerCommandedAsString = "";
                    String heightServoAsString = "";

                    if (heightServo != null) {
                        heightServoAsString = String.format("%.2f", heightServo.getPosition());
                    }
                    if (spoolMotor != null) {
                        spoolMotorPowerActualAsString = String.format("%.2f", spoolMotor.getPower());
                        spoolMotorPowerCommandedAsString = String.format("%.2f", spoolMotorPowerCommanded);
                    }

                    return heightServoAsString + "/" + spoolMotorPowerCommandedAsString + "/" + spoolMotorPowerActualAsString;
                }
            }));
        }
    }

    public void init_loop() {
        if (spoolMotor == null)
        { return; }

        spoolMotor.setDirection(spoolMotorDirection);
    }

    public void loop() {
        /// gamepad2.right_stick_y is used to deploy and retract winch (wincher-upper)
        ///
        float stickValue = Range.clip(-opMode.gamepad2.right_stick_y, -1.0f, 1.0f);
        spoolMotorPowerCommanded = throttle(opMode.gamepad2, stickValue);

        if (spoolMotor != null)
        {
            spoolMotor.setPower(spoolMotorPowerCommanded);

            /// set the deployServer to assist if we are deploying
            ///
            if (deployServo != null) {
                if (spoolMotorPowerCommanded > 0.0) {
                    deployServo.setPosition(deployServoUnwind);
                } else {
                    deployServo.setPosition(deployServoStop);
                }
            }
        }

        /// winchPosition uses gamepad2 dpad
        /// down is to lower the winch position
        /// up is to raise the winch position
        ///
        if (opMode.gamepad2.dpad_down) {
            winchHeight -= winchHeightStep;
        } else if (opMode.gamepad2.dpad_up) {
            winchHeight += winchHeightStep;
        }

        // clip the position values so that they never exceed 0..1
        winchHeight = Range.clip(winchHeight, 0, 1);

        if (heightServo != null)
        { heightServo.setPosition(winchHeight); }
    }

    // returns the list of active servos
    public List<Servo> activeServos()
    {
        List<Servo> ret = new ArrayList<Servo>();
        if (heightServo != null)
        { ret.add(heightServo); }

        if (deployServo != null)
        { ret.add(deployServo); }

        return ret;
    }

    public String getStatus()
    {
        return status;
    }
}
