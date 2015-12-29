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

    // position of the claw servo
    double clawPosition;

    // amount to change the claw servo position by
    double clawDelta = 0.01;

    DcMotorController wheelControllerFront;
    DcMotorController wheelControllerRear;
    DcMotorController grabberRackController;

    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightRear;
    DcMotor motorLeftRear;
    DcMotor grabberRack;

    Servo claw;

    Servo grabberRotator;
    static double grabberRotatorMiddle = 0.5;
    static double grabberRotatorStep = 0.5 / 180.0; // 0.5 degrees
    double grabberRotatorPosition = grabberRotatorMiddle;

    Servo continuous;
    double continuousPosition;
    static double continuousLeft = 0.0;
    static double continuousMiddle = 0.5;
    static double continuousStep = 0.05;
    static double continuousRight = 1.0;

    TelemetryDashboardAndLog dashboard;


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

        motorRightFront = hardwareMap.dcMotor.get("motor_rt_front");
        motorRightRear = hardwareMap.dcMotor.get("motor_rt_rear");
        motorLeftFront = hardwareMap.dcMotor.get("motor_lt_front");
        motorLeftRear = hardwareMap.dcMotor.get("motor_lt_rear");

        try {
            grabberRack = hardwareMap.dcMotor.get("grabber_rack");
        }
        catch (Exception E)
        {
            grabberRack = null;
        }

        List<Servo> activeServos = new ArrayList<Servo>();

        /// code to make the servos optional
        ///
        try {
            claw = hardwareMap.servo.get("claw"); // channel 6
            activeServos.add(claw);
        }
        catch (Exception E)
        {
            claw = null;
        }

        try {
            grabberRotator = hardwareMap.servo.get("grabberRotator"); // channel 1
            activeServos.add(grabberRotator);
        }
        catch (Exception E)
        {
            grabberRotator = null;
        }

        try {
            continuous = hardwareMap.servo.get("continuous");
            activeServos.add(continuous);
        }
        catch (Exception E)
        {
            continuous = null;
        }

        wheelControllerFront = hardwareMap.dcMotorController.get("wheels_front");
        wheelControllerRear = hardwareMap.dcMotorController.get("wheels_rear");

        try {
            grabberRackController = hardwareMap.dcMotorController.get("rack_controller");
        }
        catch (Exception E)
        {
            grabberRackController = null;
        }

        /// after the call to createEasyMotorController, the motor controllers
        /// work normally -- no looping mod 17 or waiting for the mode to switch
        ///
        createEasyMotorController(this, motorRightFront, motorLeftFront);
        createEasyMotorController(this, motorRightRear, motorLeftRear);
        if (grabberRackController != null) { createEasyMotorController(this, grabberRack, null); }

        /// swerve robootics interface makes the servos more synchronous
        ///
        createEasyServoController(this, activeServos);

        /// set up the dashboard with callbacks that are activated when needed
        /// supplants the telemetry object
        ///
        dashboard.addLine(dashboard.item("continuous ", new IFunc<Object>()
            {
                @Override public Double value() {return continuous.getPosition();}
            }));

        dashboard.addLine(dashboard.item("front L/R motor ", new IFunc<Object>()
        {
            @Override public String value() {return String.format("%2f", motorLeftFront.getPower()) + "/"+ String.format("%2f", motorRightFront.getPower());}
        }));

        dashboard.addLine(dashboard.item("rear L/R motor ", new IFunc<Object>()
        {
            @Override public String value() {return String.format("%2f", motorLeftRear.getPower()) + "/" + String.format("%2f", motorRightRear.getPower());}
        }));

        /// there is a chance during testing that either the grabberRack or the grabberRotator will be missing
        /// the callback (e.g. lambda) function checks if either of these is null and simply returns a blank.
        ///
        dashboard.addLine(dashboard.item("rack motor/servo ", new IFunc<Object>() {
            @Override
            public String value() {
                String grabberRackAsString = "";
                String grabberRotatorAsString = "";

                if (grabberRack != null) { grabberRackAsString = String.format("%.2f", grabberRack.getPower()); }
                if (grabberRotator != null) { grabberRotatorAsString = String.format("%.2f", grabberRotator.getPosition()); }

                return grabberRackAsString + "/" + grabberRotatorAsString;
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
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftRear.setDirection(DcMotor.Direction.REVERSE);

        // set the mode
        // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        if (grabberRack != null) { grabberRack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS); }

        clawPosition = 0.5;
        continuousPosition = continuousMiddle;
        grabberRotatorPosition = grabberRotatorMiddle;
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

    /*
     * Gamepad 1
     *
     * Gamepad 1 controls the motors via the left stick, and it controls the wrist/claw via the a,b,
     * x, y buttons
     */
        if (gamepad1.dpad_left) {
            motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            motorLeftRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            motorRightRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }
        if (gamepad1.dpad_right) {
            motorLeftFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorRightFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorLeftRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            motorRightRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        if (grabberRack != null) { grabberRack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS); }

        // throttle:  left_stick_y ranges from -1 to 1, where -1 is full up,  and 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
        float right = -gamepad1.right_stick_y;
        float left = -gamepad1.left_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // write the values to the motors
        motorRightFront.setPower(right);
        motorLeftFront.setPower(left);
        motorRightRear.setPower(right);
        motorLeftRear.setPower(left);

        /// Continuous servo
        /// X means set continuousPosition to 0.0 -- go left
        /// Y means add a delta
        /// B means set continuousPosition to 1.0 -- go right
        /// A means subtract a delta
        ///
        if (gamepad1.x) {
            continuousPosition = 0.0f;
        } else if (gamepad1.y) {
            continuousPosition += continuousStep;
        } else if (gamepad1.b) {
            continuousPosition = 1.0f;
        } else if (gamepad1.a) {
            continuousPosition -= continuousStep;
        }

        /// if we are close enough to the middle, then call it the middle
        ///
        if (Math.abs(continuousPosition - continuousMiddle) < continuousStep / 2.0)
        {
            continuousPosition = continuousMiddle;
        }

        /// using gamepad2
        /// left_stick_y means grabberRack up ranges from -1 to 1, where -1 is full up, and 1 is full down
        ///
        float grabberRackAmount = -Range.clip(gamepad2.left_stick_y, -1.0f, 1.0f);
        if (grabberRack != null) { grabberRack.setPower(grabberRackAmount); }

        /// grabberRotatorPosition using gamepad2
        /// Y means go to middle
        /// X means decrease
        /// B means increase
        ///
        if (gamepad2.y) {
            grabberRotatorPosition = grabberRotatorMiddle;
        } else if (gamepad2.x) {
            grabberRotatorPosition -= grabberRotatorStep;
        } else if (gamepad2.b) {
            grabberRotatorPosition += grabberRotatorStep;
        }

        /// if we are close enough to the middle, then call it the middle
        ///
        if (Math.abs(grabberRotatorPosition - grabberRotatorMiddle) < grabberRotatorStep / 2.0)
        {
            grabberRotatorPosition = grabberRotatorMiddle;
        }

        // clip the position values so that they never exceed 0..1
        grabberRotatorPosition = Range.clip(grabberRotatorPosition, 0, 1);
        clawPosition = Range.clip(clawPosition, 0, 1);
        continuousPosition = Range.clip(continuousPosition, 0, 1);

        // write position values to the servos
        if (grabberRotator != null) { grabberRotator.setPosition(grabberRotatorPosition); }
        if (claw != null) { claw.setPosition(clawPosition); }
        if (continuous != null) { continuous.setPosition(continuousPosition); }

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
}
