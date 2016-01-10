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

import org.griffin.apparatus.Grabber;
import org.griffin.apparatus.Winch;

import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.*;

import java.util.ArrayList;
import java.util.List;

import static org.griffin.apparatus.Util.throttle;
import static org.swerverobotics.library.ClassFactory.createEasyMotorController;
import static org.swerverobotics.library.ClassFactory.createEasyServoController;

/**
 * FourWheelDriveOp Mode
 * <p>
 * Enables control of the robot via gamepad1 and control of apparatuses via gamepad2.
 */
public class FourWheelDriveOp extends OpMode {
    DcMotorController wheelControllerLeft;
    DcMotorController wheelControllerRight;
    DcMotorController grabberRackAndWinchController;

    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightRearMotor;
    DcMotor leftRearMotor;

    Grabber grabber;
    Winch winch;

    float leftPower;
    float rightPower;

    TelemetryDashboardAndLog dashboard;
    String ComponentStatus;

    public FourWheelDriveOp()
    {
        dashboard = new TelemetryDashboardAndLog();
        ComponentStatus = "";

        grabber = new Grabber(this, "grabber_rack", DcMotor.Direction.REVERSE, "grabber_servo", dashboard, true);
        winch = new Winch(this, "winch_motor", DcMotor.Direction.FORWARD, "winch_servo", "deploy_servo", dashboard, true);

        DbgLog.msg("FourWheelDriveOp ctor");
    }
    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {

        leftFrontMotor = hardwareMap.dcMotor.get("motor_lt_front");
        leftRearMotor = hardwareMap.dcMotor.get("motor_lt_rear");
        rightFrontMotor = hardwareMap.dcMotor.get("motor_rt_front");
        rightRearMotor = hardwareMap.dcMotor.get("motor_rt_rear");

        grabber.init();
        winch.init();

        wheelControllerLeft = hardwareMap.dcMotorController.get("wheels_left");
        wheelControllerRight = hardwareMap.dcMotorController.get("wheels_right");

        try {
            grabberRackAndWinchController = hardwareMap.dcMotorController.get("rack_controller");
        }
        catch (Exception E)
        {
            DbgLog.msg("*** Exc FourWheelDriveOp." + "rack_controller " + E.getMessage());
            grabberRackAndWinchController = null;
        }

        /// after the call to createEasyMotorController, the motor controllers
        /// work normally -- no looping mod 17 or waiting for the mode to switch
        ///
        createEasyMotorController(this, rightFrontMotor, leftFrontMotor);
        createEasyMotorController(this, rightRearMotor, leftRearMotor);

        /// this is kind of ugly. just make sure that the grabber and winch share a motor controller and all is well
        ///
        if (grabberRackAndWinchController != null) { createEasyMotorController(this, grabber.rackMotor, winch.spoolMotor); }

        /// swerve robootics interface makes the servos more synchronous
        ///
        List<Servo> activeServos = new ArrayList<Servo>();
        activeServos.addAll(grabber.activeServos());
        activeServos.addAll(winch.activeServos());

        if (!activeServos.isEmpty()) { createEasyServoController(this, activeServos); }

        /// set up the dashboard with callbacks that are activated when needed
        /// supplants the telemetry object
        ///
        ComponentStatus = grabber.getStatus() + winch.getStatus();

        dashboard.addLine(dashboard.item("L/R cmd ", new IFunc<Object>()
        {
            @Override public String value() {return String.format("%.2f", leftPower) + "/"+ String.format("%.2f", rightPower);}
        }));

        dashboard.addLine(dashboard.item("L/R act ", new IFunc<Object>()
        {
            @Override public String value() {return String.format("%.2f", leftRearMotor.getPower()) + "/" + String.format("%.2f", rightRearMotor.getPower());}
        }));

        dashboard.addLine(dashboard.item(ComponentStatus, new IFunc<Object>() {
            @Override
            public String value() { return ""; }
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

        // set the mode
        // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
        leftFrontMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightFrontMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftRearMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightRearMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        grabber.init_loop();
        winch.init_loop();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {

        // tank drive
        //
        float rightStickValue = Range.clip(-gamepad1.right_stick_y, -1.0f, 1.0f);
        float leftStickValue = Range.clip(-gamepad1.left_stick_y, -1.0f, 1.0f);

        // the right bumper when held in makes the throttle lest responsive at low levels
        //
        rightPower = throttle(gamepad1, rightStickValue);
        leftPower = throttle(gamepad1, leftStickValue);

        rightFrontMotor.setPower(rightPower);
        leftFrontMotor.setPower(leftPower);
        rightRearMotor.setPower(rightPower);
        leftRearMotor.setPower(leftPower);

        grabber.loop();
        winch.loop();

        dashboard.update(); // telemetry will get updated regularly and sent back without saturating the link or processor
    }
}
