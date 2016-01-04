/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.*;

import java.util.ArrayList;
import java.util.List;

import static org.swerverobotics.library.ClassFactory.createEasyMotorController;
import static org.swerverobotics.library.ClassFactory.createEasyServoController;

/**
 * FourWheelDriveOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class FourWheelDriveOp extends OpMode {
    DcMotorController wheelControllerFront;
    DcMotorController wheelControllerRear;
    DcMotorController grabberRackAndWinchController;

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightRearMotor;
    DcMotor leftRearMotor;
    DcMotor grabberRackMotor;       // the picker-upper
    DcMotor winchMotor;             // the lifter-upper

    float leftPower;
    float rightPower;
    float grabberRackPower;
    float winchMotorPower;

    /// the winchServo raises and lowers the winch (lifter-upper)
    ///
    Servo winchServo;
    double winchPosition = winchPositionLower;
    static double winchPositionLower = 0.0;
    static double winchStep = 1.0 / 180.0; // step in 1 degree increments

    Servo grabberRotatorServo;
    double grabberRotatorPosition = grabberRotatorMiddle;
    static double grabberRotatorMiddle = 0.2; //0.5;
    static double grabberRotatorStep = 0.025; //5.0 / 180.0; // 0.5 degrees


    /// I found this on the web. Don't know if it is true -JGM
    ///
    //    Given that the HiTechic Servo Controller allows setting of the PWM output from 750 to 2250 microseconds
    //    with a step resolution of 5.88 microseconds anything under servo[foo] = 42 puts the servo into the zombie state.
    //    The math is the same for the other end of the scale (212).
    //
    //    Anything in between those two values will fall somewhere within the 1260 degree rotation of the servo.
    //
    Servo continuousServo;
    double continuousServoMovement;
    static double continuousRotateLeft = 43.0 / 256.0;
    static double continuousMiddle = 0.5;
    static double continuousRotateRight = 211.0 / 256.0;
    static double continuousRotateStep = 0.1;

    TelemetryDashboardAndLog dashboard;
    String ComponentStatus;


    public FourWheelDriveOp()
    {
        DbgLog.msg("FourWheelDriveOp");
    }
    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
        dashboard = new TelemetryDashboardAndLog();
        ComponentStatus = "";

        rightFrontMotor = hardwareMap.dcMotor.get("motor_rt_front");
        rightRearMotor = hardwareMap.dcMotor.get("motor_rt_rear");
        leftFrontMotor = hardwareMap.dcMotor.get("motor_lt_front");
        leftRearMotor = hardwareMap.dcMotor.get("motor_lt_rear");

        try {
            grabberRackMotor = hardwareMap.dcMotor.get("grabber_rack");
            ComponentStatus += "G";
        }
        catch (Exception E)
        {
            grabberRackMotor = null;
            ComponentStatus += "-";
        }

        try {
            winchMotor = hardwareMap.dcMotor.get("winch_motor");
            ComponentStatus += "W";
        }
        catch (Exception E)
        {
            winchMotor = null;
            ComponentStatus += "-";
        }

        List<Servo> activeServos = new ArrayList<Servo>();

        /// code to make the servos optional
        ///
        try {
            winchServo = hardwareMap.servo.get("winch_servo"); // channel 6
            activeServos.add(winchServo);
            ComponentStatus += "w";
        }
        catch (Exception E)
        {
            winchServo = null;
            ComponentStatus += "-";
        }

        try {
            grabberRotatorServo = hardwareMap.servo.get("grabberRotator"); // channel 1
            activeServos.add(grabberRotatorServo);
            ComponentStatus += "g";
        }
        catch (Exception E)
        {
            grabberRotatorServo = null;
            ComponentStatus += "-";
        }

        try {
            continuousServo = hardwareMap.servo.get("continuous");
            activeServos.add(continuousServo);
            ComponentStatus += "C";
        }
        catch (Exception E)
        {
            continuousServo = null;
            ComponentStatus += "-";
        }

        wheelControllerFront = hardwareMap.dcMotorController.get("wheels_front");
        wheelControllerRear = hardwareMap.dcMotorController.get("wheels_rear");

        try {
            grabberRackAndWinchController = hardwareMap.dcMotorController.get("rack_controller");
        }
        catch (Exception E)
        {
            grabberRackAndWinchController = null;
        }

        /// after the call to createEasyMotorController, the motor controllers
        /// work normally -- no looping mod 17 or waiting for the mode to switch
        ///
        createEasyMotorController(this, rightFrontMotor, leftFrontMotor);
        createEasyMotorController(this, rightRearMotor, leftRearMotor);
        if (grabberRackAndWinchController != null) { createEasyMotorController(this, grabberRackMotor, winchMotor); }

        /// swerve robootics interface makes the servos more synchronous
        ///
        createEasyServoController(this, activeServos);

        /// set up the dashboard with callbacks that are activated when needed
        /// supplants the telemetry object
        ///
        if (continuousServo == null)
        {
            dashboard.addLine(dashboard.item(ComponentStatus, new IFunc<Object>() {
                @Override
                public String value() { return ""; }}));
        }
        else {
            dashboard.addLine(dashboard.item(ComponentStatus + ": continuous ", new IFunc<Object>() {
                @Override
                public String value() {
                    return String.format("%.2f", continuousServo.getPosition());
                }
            }));
        }

        dashboard.addLine(dashboard.item("front L/R motor ", new IFunc<Object>()
        {
            @Override public String value() {return String.format("%.2f", leftPower) + "/"+ String.format("%.2f", rightPower);}
        }));

        dashboard.addLine(dashboard.item("rear L/R motor ", new IFunc<Object>()
        {
            @Override public String value() {return String.format("%.2f", leftRearMotor.getPower()) + "/" + String.format("%.2f", rightRearMotor.getPower());}
        }));

        /// there is a chance during testing that either the grabberRack or the grabberRotator will be missing
        /// the callback (e.g. lambda) function checks if either of these is null and simply returns a blank.
        ///
        dashboard.addLine(dashboard.item("grabber servo/motor/enc/L/U ", new IFunc<Object>() {
            @Override
            public String value() {
                String grabberRackAsString = "";
                String grabberRotatorAsString = "";
                String grabberEncoderAsString = "";

                if (grabberRotatorServo != null) { grabberRotatorAsString = String.format("%.2f", grabberRotatorServo.getPosition()); }
                if (grabberRackMotor != null) {
                    grabberRackAsString = String.format("%.2f", grabberRackPower);
                    grabberEncoderAsString = String.format("%d", grabberRackMotor.getCurrentPosition());
                }

                return grabberRotatorAsString + "/" + grabberRackAsString + "/" + grabberEncoderAsString + "/" + grabberRackLowerEncoder + "/" + grabberRackUpperEncoder;
            }
        }));

        /// there is a chance during testing that either the grabberRack or the grabberRotator will be missing
        /// the callback (e.g. lambda) function checks if either of these is null and simply returns a blank.
        ///
        dashboard.addLine(dashboard.item("winch motor/servo ", new IFunc<Object>() {
            @Override
            public String value() {
                String winchMotorAsString = "";
                String winchServoAsString = "";

                if (winchMotor != null) { winchMotorAsString = String.format("%.2f", winchMotorPower); }
                if (winchServo != null) { winchServoAsString = String.format("%.2f", winchServo.getPosition()); }

                return winchMotorAsString + "/" + winchServoAsString;
            }
        }));

        DbgLog.msg("Four Wheel Drive Initialized");
    }

    /*
     * Code that runs repeatedly when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init_loop()
     */
    @Override
    public void init_loop() {

        /// only the motor on the left side runs in reverse
        ///
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        if (grabberRackMotor != null) { grabberRackMotor.setDirection(DcMotor.Direction.REVERSE); }

        // set the mode
        // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
        leftFrontMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightFrontMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftRearMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightRearMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        /// zero the encoder so things don't move on startup
        ///
        if (grabberRackMotor != null) { grabberRackMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS); }

        winchPosition = winchPositionLower;
        continuousServoMovement = continuousMiddle;
        grabberRotatorPosition = grabberRotatorMiddle;
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

    //    if (grabberRackMotor != null) { grabberRackMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS); }
    /*
     * Gamepad 1
     *
     * Gamepad 1 controls the motors via the left stick, and it controls the wrist/claw via the a,b,
     * x, y buttons
     */
//        if (gamepad1.dpad_left) {
//            motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//            motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//            motorLeftRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//            motorRightRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
//        }
//        if (gamepad1.dpad_right) {
//            motorLeftFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//            motorRightFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//            motorLeftRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//            motorRightRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
//        }

//        if (grabberRack != null) { grabberRack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS); }

        // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
        rightPower = -gamepad1.right_stick_y;
        leftPower = -gamepad1.left_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        rightPower = Range.clip(rightPower, -1, 1);
        leftPower = Range.clip(leftPower, -1, 1);

        /// if the right bumper is being held in, then allow more precise control of throttle
        /// by making significantly less response at lower throttle positions (low gain)
        float gain = gainHigh;
        if (gamepad1.right_bumper)
        { gain = gainLow; }

        rightPower = throttle(rightPower, gain);
        leftPower = throttle(leftPower, gain);

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
            if (gamepad1.left_bumper)
            { continuousServoMovement -= continuousRotateStep; }
            else
            { continuousServoMovement += continuousRotateStep; }
        }

        /// using gamepad2
        ///
        DoGrabberRack();

        /// if the right bumper is being held in, then allow more precise control of rack
        /// by making significantly less response at lower throttle positions (low gain)
        ///
        gain = gainHigh;
        if (gamepad2.right_bumper)
        { gain = gainLow; }

        /// gamepad2.left_stick_y is used to raise and lower the rack on the grabber (picker-upper)
        ///
        grabberRackPower = Range.clip(-gamepad2.left_stick_y, -1.0f, 1.0f);
        grabberRackPower = throttle(grabberRackPower, gain);

        /// gamepad2.right_stick_y is used to deploy and retract the winch (lifter-upper)
        ///
        winchMotorPower = Range.clip(-gamepad2.right_stick_y, -1.0f, 1.0f);
        winchMotorPower = throttle(winchMotorPower, gain);

        /// grabberRotatorPosition using gamepad2
        /// Y means go to middle
        /// X means decrease
        /// B means increase
        ///
//        if (gamepad2.y) {
//            grabberRotatorPosition = grabberRotatorMiddle;
//        } else if (gamepad2.x) {
//            grabberRotatorPosition -= grabberRotatorStep;
//        } else if (gamepad2.b) {
//            grabberRotatorPosition += grabberRotatorStep;
//        }

        /// grabberRotatorPosition using gamepad2
        /// add up the values of the triggers and set to deflection from the middle
        ///
        float leftTrigger = -gamepad2.left_trigger / 2.0f;
        float rightTrigger = gamepad2.right_trigger / 2.0f;
        grabberRotatorPosition = leftTrigger + rightTrigger + grabberRotatorMiddle;

        /// if we are close enough to the middle, then call it the middle
        ///
        if (Math.abs(grabberRotatorPosition - grabberRotatorMiddle) < grabberRotatorStep / 2.0)
        {
            grabberRotatorPosition = grabberRotatorMiddle;
        }

        /// winchPosition ujsing gamepad2 dpad
        /// down is to lower the winch position
        /// up is to raise the winch position
        ///
        if (gamepad2.dpad_down) {
            winchPosition -= winchStep;
        } else if (gamepad2.dpad_up) {
            winchPosition += winchStep;
        }

        // clip the position values so that they never exceed 0..1
        grabberRotatorPosition = Range.clip(grabberRotatorPosition, 0, 1);
        winchPosition = Range.clip(winchPosition, 0, 1);
        continuousServoMovement = Range.clip(continuousServoMovement, 0, 1);

        /// write position values to the servos
        ///
        if (grabberRotatorServo != null) { grabberRotatorServo.setPosition(grabberRotatorPosition); }
        if (winchServo != null) { winchServo.setPosition(winchPosition); }
        if (continuousServo != null) { continuousServo.setPosition(continuousServoMovement); }

        /// write the motor powers
        ///
        if (grabberRackMotor != null) {
            if (grabberRackMotor.getMode() == DcMotorController.RunMode.RUN_TO_POSITION)
            {
                grabberRackMotor.setPower(1.0f);    // get into position as fast as possible

                if (gamepad2.a)
                { grabberRackMotor.setTargetPosition(grabberRackLowerEncoder); }
                else if (gamepad2.y)
                { grabberRackMotor.setTargetPosition(grabberRackUpperEncoder); }
            }
            else
            {
                grabberRackMotor.setPower(grabberRackPower);
            }
        }
        if (winchMotor != null) { winchMotor.setPower(winchMotorPower); }

        rightFrontMotor.setPower(rightPower);
        leftFrontMotor.setPower(leftPower);
        rightRearMotor.setPower(rightPower);
        leftRearMotor.setPower(leftPower);

//    /*
//     * Gamepad 2
//     *
//     * Gamepad controls the motors via the right trigger as a throttle, left trigger as reverse, and
//     * the left stick for direction. This type of control is sometimes referred to as race car mode.
//     */
//
//            // we only want to process gamepad2 if someone is using one of it's analog inputs. If you always
//            // want to process gamepad2, remove this check
//      /*
//      if (gamepad2.atRest() == false) {
//
//        // throttle is taken directly from the right trigger, the right trigger ranges in values from
//        // 0 to 1
//        throttle = gamepad2.right_trigger;
//
//        // if the left trigger is pressed, go in reverse
//        if (gamepad2.left_trigger != 0.0) {
//          throttle = -gamepad2.left_trigger;
//        }
//
//        // assign throttle to the left and right motors
//        right = throttle;
//        left = throttle;
//
//        // now we need to apply steering (direction). The left stick ranges from -1 to 1. If it is
//        // negative we want to slow down the left motor. If it is positive we want to slow down the
//        // right motor.
//        if (gamepad2.left_stick_x < 0) {
//          // negative value, stick is pulled to the left
//          left = left * (1 + gamepad2.left_stick_x);
//        }
//        if (gamepad2.left_stick_x > 0) {
//          // positive value, stick is pulled to the right
//          right = right * (1 - gamepad2.left_stick_x);
//        }
//
//        // write the values to the motor. This will over write any values placed while processing gamepad1
//        motorRightFront.setPower(right);
//        motorLeftFront.setPower(left);
//        motorRightRear.setPower(right);
//        motorLeftRear.setPower(left);
//      } */
        dashboard.update(); // telemetry will get updated regularly and sent back without saturating the link or processor
    }

    int grabberRackLowerEncoder = 0;
    int grabberRackUpperEncoder = 0;
    int grabberRackUpDownFactor = 1;   // set to -1 if the encoder reads smaller values when rack is extended

    private void DoGrabberRack()
    {
        if (grabberRackMotor == null)
        { return; }

        /// if the "x" button is down and the left bumper is pressed, then run the rack
        /// without encoders. when the left trigger gets pressed, use the
        /// current encoder value as the low. when the right trigger gets pressed
        /// use the current encoder value as the high
        ///
        if (gamepad2.x)
        {
            grabberRackMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

            /// when they press the left bumper, use the current encoder value as the lower
            ///
            if (gamepad2.left_bumper)
            { grabberRackLowerEncoder = grabberRackMotor.getCurrentPosition() * grabberRackUpDownFactor; }

            /// when they press the right bumper, use the current encoder value as the upper
            ///
            if (gamepad2.right_bumper)
            { grabberRackUpperEncoder = grabberRackMotor.getCurrentPosition() * grabberRackUpDownFactor; }
        }
        else
        {
            grabberRackMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        }
    }
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
    static float joystickDeadband = 0.009f;     // joystick readings less than this should be treated as 0.0

    static float gainHigh = 0.95f;
    static float gainLow = 0.3f;

    private float throttle(float joystickAmount, float gain)
    {
        /// save the sign of the joystick and use the absolute value -- this function
        /// only works for values >0
        ///
        float sign = Math.signum(joystickAmount);
        joystickAmount = Math.abs(joystickAmount);

        /// cube term without calling the Math.pow function
        ///
        float temp = (joystickAmount-joystickDeadband)/(1.0f-joystickDeadband);
        temp *= temp;
        temp *= temp;

        float ret = gain * (temp + (1.0f - gain) * temp);
        return Math.max(ret, 0.0f) * sign;
    }
}
