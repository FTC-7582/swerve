package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

import static org.swerverobotics.library.ClassFactory.createEasyMotorController;

/**
 * This is a fun little autonomous program designed to have the robot
 * do a happy dance.
 */
@TeleOp(name="Happy Dance OpMode")
public class HappyDanceOp extends SynchronousOpMode
{
    /* Declare here any fields you might find useful. */
    DcMotor[] drive = new DcMotor[4];
    DcMotor pinion;
    Servo grabber;

    //DcMotor motorLeft;
    //DcMotor motorRight;

    @Override public void main() throws InterruptedException
    {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */

        //Register the drive motors to an array
        //0 and 1: front, 2 and 3: rear, 0 and 2: left, 1 and 3: right
        this.drive[0] = this.hardwareMap.dcMotor.get("motor_lt_front");
        this.drive[1] = this.hardwareMap.dcMotor.get("motor_rt_front");
        this.drive[2] = this.hardwareMap.dcMotor.get("motor_lt_rear");
        this.drive[3] = this.hardwareMap.dcMotor.get("motor_rt_rear");

        pinion = this.hardwareMap.dcMotor.get("grabber_rack");

        grabber = this.hardwareMap.servo.get("grabber_servo");

        createEasyMotorController(this, drive[2], drive[0]);
        createEasyMotorController(this, drive[3], drive[1]);

        // Wait for the game to start
        waitForStart();

        pinion.setPower(-0.5f);
        Thread.sleep(500);
        pinion.setPower(0);
        grabber.setPosition(0.5d);

        forward(0.01f, 10);

        //Wait for gamepad 1's right bumper to be pressed
        //while (!gamepad1.right_bumper) {Thread.sleep(10);}

        // Go go gadget robot!
        while (opModeIsActive())
        {
            forward(0.3f, 500);
            backward(0.3f, 500);
            left(0.3f, 750);
            right(0.45f, 625);
            pinion.setPower(0.5f);
            Thread.sleep(300);
            pinion.setPower(0);
            grabber.setPosition(1);
            Thread.sleep(300);
            grabber.setPosition(0.5f);
            Thread.sleep(600);
            grabber.setPosition(0);
            Thread.sleep(300);
            pinion.setPower(-0.5f);
            Thread.sleep(300);
            pinion.setPower(0);

            if (updateGamepads())
            {
//                drive[0].setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
//                drive[1].setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
//                drive[2].setPower(-Range.clip(gamepad1.right_stick_y, -1, 1));
//                drive[3].setPower(-Range.clip(gamepad1.right_stick_y, -1, 1));
                // The game pad state has changed. Do something with that!
            }

            telemetry.update();
            idle();
        }
    }

    public void forward(long time) throws InterruptedException{
        drive[0].setPower(-1);
        drive[2].setPower(-1);
        drive[1].setPower(1);
        drive[3].setPower(1);

        Thread.sleep(time);

        for (DcMotor mtr : drive){
            mtr.setPower(0);
        }
    }
    public void forward(float power, long time) throws InterruptedException{
        //power*=0.5f;
        drive[0].setPower(-power);
        drive[2].setPower(-power);
        drive[1].setPower(power);
        drive[3].setPower(power);

        Thread.sleep(time);

        for (DcMotor mtr : drive) {
            mtr.setPower(0);
        }
    }

    public void backward(long time) throws InterruptedException{
        drive[0].setPower(1);
        drive[2].setPower(1);
        drive[1].setPower(-1);
        drive[3].setPower(-1);

        Thread.sleep(time);

        for (DcMotor mtr : drive){
            mtr.setPower(0);
        }
    }
    public void backward(float power, long time) throws InterruptedException{
        drive[0].setPower(power);
        drive[2].setPower(power);
        drive[1].setPower(-power);
        drive[3].setPower(-power);

        Thread.sleep(time);

        for (DcMotor mtr : drive){
            mtr.setPower(0);
        }
    }
    public void left(long time) throws InterruptedException{
        for (DcMotor mtr : drive){
            mtr.setPower(1);
        }

        Thread.sleep(time);

        for (DcMotor mtr : drive){
            mtr.setPower(0);
        }
    }
    public void left(float power, long time) throws InterruptedException{
        for (DcMotor mtr : drive){
            mtr.setPower(power);
        }

        Thread.sleep(time);

        for (DcMotor mtr : drive){
            mtr.setPower(0);
        }
    }
    public void right(long time) throws InterruptedException{
        for (DcMotor mtr : drive){
            mtr.setPower(-1);
        }

        Thread.sleep(time);

        for (DcMotor mtr : drive){
            mtr.setPower(0);
        }
    }
    public void right(float power, long time) throws InterruptedException{
        for (DcMotor mtr : drive){
            mtr.setPower(-power);
        }

        Thread.sleep(time);

        for (DcMotor mtr : drive){
            mtr.setPower(0);
        }
    }

    private class Oscillate extends Thread{
        public Oscillate(){
            while (true){
                grabber.setPosition(0);
                try {
                    Thread.sleep(100);
                } catch(InterruptedException e){}
                grabber.setPosition(1);
            }
        }
    }
}
