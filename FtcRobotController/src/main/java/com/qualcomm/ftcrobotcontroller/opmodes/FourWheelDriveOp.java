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

    // position of the wrist servo
    double wristPosition;

    // amount to change the wrist servo position by
    double wristDelta = 0.01;

    // command for the continuous servo
    double continuousPosition;
    static double continuousStop = 0.5;
    double continuousDelta = 0.05;

    DcMotorController.DeviceMode devMode;
    DcMotorController wheelControllerFront;
    DcMotorController wheelControllerRear;
    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightRear;
    DcMotor motorLeftRear;

    Servo claw;
    Servo wrist;
    Servo continuous;

    int numOpLoops = 1;

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
        motorRightFront = hardwareMap.dcMotor.get("motor_rt_front");
        motorRightRear = hardwareMap.dcMotor.get("motor_rt_rear");
        motorLeftFront = hardwareMap.dcMotor.get("motor_lt_front");
        motorLeftRear = hardwareMap.dcMotor.get("motor_lt_rear");

        /// code to make the servos optional
        ///
        try {
            claw = hardwareMap.servo.get("claw"); // channel 6
        }
        catch (Exception E)
        {
            claw = null;
        }

        try {
            wrist = hardwareMap.servo.get("wrist"); // channel 1
        }
        catch (Exception E)
        {
            wrist = null;
        }

        try {
            continuous = hardwareMap.servo.get("continuous");
        }
        catch (Exception E)
        {
            continuous = null;
        }

        wheelControllerFront = hardwareMap.dcMotorController.get("wheels_front");
        wheelControllerRear = hardwareMap.dcMotorController.get("wheels_rear");

        DbgLog.msg("Four Wheel Drive Initialized");
    }

    /*
     * Code that runs repeatedly when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init_loop()
     */
    @Override
    public void init_loop() {

        devMode = DcMotorController.DeviceMode.WRITE_ONLY;

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

        wristPosition = 0.6;
        clawPosition = 0.5;
        continuousPosition = continuousStop;
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        // The op mode should only use "write" methods (setPower, setMode, etc) while in
        // WRITE_ONLY mode or SWITCHING_TO_WRITE_MODE
        if (allowedToWrite()) {
    /*
     * Gamepad 1
     *
     * Gamepad 1 controls the motors via the left stick, and it controls the wrist/claw via the a,b,
     * x, y buttons
     */

            if (gamepad1.dpad_left) {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorLeftRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
                motorRightRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            }
            if (gamepad1.dpad_right) {
                // Nxt devices start up in "write" mode by default, so no need to switch modes here.
                motorLeftFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                motorRightFront.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                motorLeftRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
                motorRightRear.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            }

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

            //Continuous servo
            if (continuous != null) {
                if (gamepad1.x) {
                    continuous.setPosition(0.0f);
                } else if (gamepad1.y) {
                    continuous.setPosition(0.5f);
                } else if (gamepad1.b) {
                    continuous.setPosition(1.0f);
                }
            }

            // update the position of the wrist
//      if (gamepad1.a) {
//        wristPosition -= wristDelta;
//      }
//
//      if (gamepad1.y) {
//        wristPosition += wristDelta;
//      }
//
//      // update the position of the claw
//      if (gamepad1.x) {
//        continuousPosition -= continuousDelta;
//        clawPosition -= clawDelta;
//      }
//
//      if (gamepad1.b) {
//        continuousPosition += continuousDelta;
//        clawPosition += clawDelta;
//      }

            // clip the position values so that they never exceed 0..1
            wristPosition = Range.clip(wristPosition, 0, 1);
            clawPosition = Range.clip(clawPosition, 0, 1);
            continuousPosition = Range.clip(continuousPosition, 0, 1);

            // if we get outside the interval
            if (Math.abs(continuousPosition - continuousStop) < continuousDelta / 2.0)
            {
                continuousPosition = continuousStop;
            }

            // write position values to the wrist and claw servo
            if (wrist != null) { wrist.setPosition(wristPosition); }
            if (claw != null) { claw.setPosition(clawPosition); }

    /*
     * Gamepad 2
     *
     * Gamepad controls the motors via the right trigger as a throttle, left trigger as reverse, and
     * the left stick for direction. This type of control is sometimes referred to as race car mode.
     */

            // we only want to process gamepad2 if someone is using one of it's analog inputs. If you always
            // want to process gamepad2, remove this check
      /*
      if (gamepad2.atRest() == false) {

        // throttle is taken directly from the right trigger, the right trigger ranges in values from
        // 0 to 1
        throttle = gamepad2.right_trigger;

        // if the left trigger is pressed, go in reverse
        if (gamepad2.left_trigger != 0.0) {
          throttle = -gamepad2.left_trigger;
        }

        // assign throttle to the left and right motors
        right = throttle;
        left = throttle;

        // now we need to apply steering (direction). The left stick ranges from -1 to 1. If it is
        // negative we want to slow down the left motor. If it is positive we want to slow down the
        // right motor.
        if (gamepad2.left_stick_x < 0) {
          // negative value, stick is pulled to the left
          left = left * (1 + gamepad2.left_stick_x);
        }
        if (gamepad2.left_stick_x > 0) {
          // positive value, stick is pulled to the right
          right = right * (1 - gamepad2.left_stick_x);
        }

        // write the values to the motor. This will over write any values placed while processing gamepad1
        motorRightFront.setPower(right);
        motorLeftFront.setPower(left);
        motorRightRear.setPower(right);
        motorLeftRear.setPower(left);
      } */
        }

        // To read any values from the NXT controllers, we need to switch into READ_ONLY mode.
        // It takes time for the hardware to switch, so you can't switch modes within one loop of the
        // op mode. Every 17th loop, this op mode switches to READ_ONLY mode, and gets the current power.
        if (numOpLoops % 17 == 0){
            // Note: If you are using the NxtDcMotorController, you need to switch into "read" mode
            // before doing a read, and into "write" mode before doing a write. This is because
            // the NxtDcMotorController is on the I2C interface, and can only do one at a time. If you are
            // using the USBDcMotorController, there is no need to switch, because USB can handle reads
            // and writes without changing modes. The NxtDcMotorControllers start up in "write" mode.
            // This method does nothing on USB devices, but is needed on Nxt devices.

            syncSetMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
        }

        // Every 17 loops, switch to read mode so we can read data from the NXT device.
        // Only necessary on NXT devices.
        if (wheelControllerFront.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {

            // Update the reads after some loops, when the command has successfully propagated through.

            telemetry.addData("continuous ", continuousPosition);
            telemetry.addData("front L/R  motor ", Double.toString(motorLeftFront.getPower()) + "/"+Double.toString(motorRightFront.getPower()));
            telemetry.addData("rear L/R rear motor  ", Double.toString(motorLeftRear.getPower()) + "/" + motorRightRear.getPower());
            telemetry.addData("RunMode: ", motorLeftFront.getMode().toString());

            /// looks to me like setting the DeviceMode to WRITE_ONLY takes some time
            /// this won't return until the transition is complete
            ///
            int delayms = syncSetMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

            telemetry.addData("Status OPL/Delay ", String.valueOf(numOpLoops) + " / " + String.valueOf(delayms));
            // Reset the loop
            // numOpLoops = 0;
        }

        // Update the current devMode
        devMode = wheelControllerFront.getMotorControllerDeviceMode();
        numOpLoops++;
    }

    // If the device is in either of these two modes, the op mode is allowed to write to the HW.
    private boolean allowedToWrite(){
        return (devMode == DcMotorController.DeviceMode.WRITE_ONLY);
    }

    /// the DcMotorControllers take a while to switch modes. we can't afford to have
    /// have the controller to switch from WRITE_ONLY to READ_ONLY in the middle of
    /// move or (less importantly) switch from READ_ONLY to WRITE_ONLY in the middle
    /// of gathering telemetry. Use this method to change the mode and not return until
    /// the transition is done. It checks every 10ms for a transition
    ///
    private int syncSetMotorControllerDeviceMode(DcMotorController.DeviceMode newMode)
    {
        DcMotorController.DeviceMode devModeFront;
        DcMotorController.DeviceMode devModeRear;
        int delay = 0;

        while (true)
        {
            wheelControllerFront.setMotorControllerDeviceMode(newMode);
            wheelControllerRear.setMotorControllerDeviceMode(newMode);

            /// sleep for 10ms and check
            ///
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                /// sleep got interrupted - oh well
                e.printStackTrace();
            }
            delay++;

            devModeFront = wheelControllerFront.getMotorControllerDeviceMode();
            devModeRear = wheelControllerRear.getMotorControllerDeviceMode();

            if (devModeFront == newMode && devModeRear == newMode)
            { break; }
        }

        return delay * 10;
    }
}
