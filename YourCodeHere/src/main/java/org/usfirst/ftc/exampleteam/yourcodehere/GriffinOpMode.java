package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

import static org.swerverobotics.library.ClassFactory.createEasyMotorController;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="Griffin OpMode")
public class GriffinOpMode extends SynchronousOpMode
{
    /* Declare here any fields you might find useful. */
    DcMotor[] drive = new DcMotor[4];

    //DcMotor motorLeft;
    //DcMotor motorRight;

    @Override public void main() throws InterruptedException
    {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */

        this.drive[0] = this.hardwareMap.dcMotor.get("motor_lt_front");
        this.drive[1] = this.hardwareMap.dcMotor.get("motor_rt_front");
        this.drive[2] = this.hardwareMap.dcMotor.get("motor_lt_rear");
        this.drive[3] = this.hardwareMap.dcMotor.get("motor_rt_rear");

        createEasyMotorController(this, drive[2], drive[0]);
        createEasyMotorController(this, drive[3], drive[1]);

        // Wait for the game to start
        waitForStart();

        forward(0.5f, 1000);
        backward(0.5f, 1000);
        left(0.25f, 500);
        forward(0.5f, 750);
        right(0.5f, 1000);

        //Wait for gamepad 1's right bumper to be pressed
        while (!gamepad1.right_bumper) {Thread.sleep(10);}

        // Go go gadget robot!
        while (opModeIsActive())
        {
            if (updateGamepads())
            {
                drive[0].setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
                drive[1].setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
                drive[2].setPower(-Range.clip(gamepad1.right_stick_y, -1, 1));
                drive[3].setPower(-Range.clip(gamepad1.right_stick_y, -1, 1));
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
}
