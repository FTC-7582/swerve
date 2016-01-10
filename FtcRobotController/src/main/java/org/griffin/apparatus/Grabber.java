package org.griffin.apparatus;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.IFunc;

import java.util.ArrayList;
import java.util.List;

import static org.griffin.apparatus.Util.throttle;

/**
 * Created by Ultimate on 1/4/2016.
 *
 * Grabber is composed of two items - a rack and a servo.
 * The rack is motor driven with an encoder.
 * The servo is used to dump the grabber
 */
public class Grabber {
    public Servo rotatorServo;
    private static double rotatorServoMiddle = 0.5;
    // we need full range from -45 to 225 (-pi/4 to 5*pi/4)
    // from center we need to move -135 and +135 from 90
    // with a 2:1 gear ratio, the move is -67.5 and +67.5 from 90
    // or in radians about 1.178
    private static float rotatorServoScale = 90.0f / 180.0f;
    private static double rotatorServoStep = 0.025; //5.0 / 180.0; // 0.5 degrees

    public DcMotor rackMotor;       // the picker-upper
    private double rackMotorPowerCommanded;
    private int rackMotorEncLowerPosition;
    private int rackMotorEncUpperPosition;

    private String rackMotorName;
    private String rotatorServoName;

    private DcMotor.Direction rackMotorDirection;
    private String status;

    private TelemetryDashboardAndLog dashboard;
    private boolean logging;

    private OpMode opMode;

    public Grabber(OpMode opMode, String motorName, DcMotor.Direction motorDirection, String rotatorName, TelemetryDashboardAndLog dashboard, boolean logging)
    {
        this.opMode = opMode;

        this.dashboard = dashboard;
        this.rackMotorName = motorName;
        this.rackMotorDirection = motorDirection;
        this.rotatorServoName = rotatorName;
        this.logging = logging;
    }

    public void init() {
        status = "";

        try {
            rackMotor = opMode.hardwareMap.dcMotor.get(rackMotorName);
            status += "G";
        } catch (Exception E) {
            DbgLog.msg("*** Exc Grabber." + rackMotorName + " " + E.getMessage());
            rackMotor = null;
            status += "-";
        }

        try {
            rotatorServo = opMode.hardwareMap.servo.get(rotatorServoName);
            status += "g";
        } catch (Exception E) {
            DbgLog.msg("*** Exc Grabber." + rotatorServoName + " " + E.getMessage());
            rotatorServo = null;
            status += "-";
        }

        if (logging) {
            dashboard.addLine(dashboard.item("Grabber servo/motor/mode/enc/L/U ", new IFunc<Object>() {
                @Override
                public String value() {
                    String rackMotorPowerCommandedAsString = "";
                    String rotatorServoAsString = "";
                    String grabberEncoderAsString = "";
                    String rackMotorModeAsString = "";

                    if (rotatorServo != null) {
                        rotatorServoAsString = String.format("%.2f", rotatorServo.getPosition());
                    }
                    if (rackMotor != null) {
                        rackMotorPowerCommandedAsString = String.format("%.2f", rackMotorPowerCommanded);
                        grabberEncoderAsString = String.format("%d", rackMotor.getCurrentPosition());
                        switch (rackMotor.getMode()) {
                            case RUN_TO_POSITION:
                                rackMotorModeAsString = "ToP";
                                break;

                            case RESET_ENCODERS:
                                rackMotorModeAsString = "RST";
                                break;

                            case RUN_USING_ENCODERS:
                                rackMotorModeAsString = "+EN";
                                break;

                            default:
                                rackMotorModeAsString = "-EN";
                        }
                    }

                    return rotatorServoAsString + "/" + rackMotorPowerCommandedAsString + "/" + rackMotorModeAsString + "/" +
                            grabberEncoderAsString + "/" + rackMotorEncLowerPosition + "/" + rackMotorEncUpperPosition;
                }
            }));
        }
    }

    public void init_loop()
    {
        if (rackMotor == null)
        { return; }

        rackMotor.setDirection(rackMotorDirection);

        /// zero the encoder so things don't move on startup
        ///
        rackMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        /// set the up and down so the test in moveRackMotor always fails until both encoder values are set
        ///
        rackMotorEncLowerPosition = 50000;
        rackMotorEncUpperPosition = -50000;
    }

    private static int grabberRackUpDownFactor = 1;   // set to -1 if the encoder reads smaller values when rack is extended

    public void loop()
    {
        if (rackMotor == null)
        { return; }

        /// if the "x" button is down and the left bumper is pressed, then run the rack
        /// using encoders but not run to position. when the left trigger gets pressed, use the
        /// current encoder value as the low. when the right trigger gets pressed
        /// use the current encoder value as the high
        ///
        if (opMode.gamepad2.x)
        {
            rackMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

            /// when they press the left bumper, use the current encoder value as the lower
            ///
            if (opMode.gamepad2.left_bumper)
            { rackMotorEncLowerPosition = rackMotor.getCurrentPosition() * grabberRackUpDownFactor; }

            /// when they press the right bumper, use the current encoder value as the upper
            ///
            if (opMode.gamepad2.right_bumper)
            { rackMotorEncUpperPosition = rackMotor.getCurrentPosition() * grabberRackUpDownFactor; }
        }
        else
        {
            rackMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }

        /// gamepad2.left_stick_y is used to raise and lower the rack on the grabber (picker-upper)
        ///
        float stickValue = Range.clip(-opMode.gamepad2.left_stick_y, -1.0f, 1.0f);
        float motorThrottle = throttle(opMode.gamepad2, stickValue);

        if (rackMotor != null)
        { moveRackMotor(motorThrottle); }


        /// for the rotatorServer, use the triggers -- left trigger means rotate left, etc.
        /// add up the values of the triggers and set to deflection from the middle
        ///
        float leftTrigger = -opMode.gamepad2.left_trigger / 2.0f * rotatorServoScale;
        float rightTrigger = opMode.gamepad2.right_trigger / 2.0f * rotatorServoScale;
        double rotatorServoPositionCommanded = leftTrigger + rightTrigger + rotatorServoMiddle;

        /// if we are close enough to the middle, then call it the middle
        ///
        if (Math.abs(rotatorServoPositionCommanded - rotatorServoMiddle) < rotatorServoStep)
        {
            rotatorServoPositionCommanded = rotatorServoMiddle;
        }

        if (rotatorServo != null)
        { rotatorServo.setPosition(rotatorServoPositionCommanded); }
    }

    private void moveRackMotor(float motorThrottle)
    {
        if (rackMotor.getMode() == DcMotorController.RunMode.RUN_TO_POSITION)
        {
            // if the the lower position is bigger than the upper, encoder
            // positions have probably not been set yet -- do nothing
            //
            if (rackMotorEncLowerPosition > rackMotorEncUpperPosition)
            { return; }

            if (opMode.gamepad2.a)
            {
                rackMotor.setPower(1.0f);    // get into position as fast as possible
                rackMotor.setTargetPosition(rackMotorEncLowerPosition);
            }
            else if (opMode.gamepad2.y)
            {
                rackMotor.setPower(1.0f);    // get into position as fast as possible
                rackMotor.setTargetPosition(rackMotorEncUpperPosition);
            }
        }
        else
        {
            rackMotor.setPower(motorThrottle);
        }
    }

    // returns the list of active servos
    public List<Servo> activeServos()
    {
        List<Servo> ret = new ArrayList<Servo>();
        if (rotatorServo != null)
        { ret.add(rotatorServo); }

        return ret;
    }

    public String getStatus()
    {
        return status;
    }
}
