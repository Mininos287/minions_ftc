package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="test_motor", group ="test" )
public class TestMotor extends OpMode {

    private DcMotor left_back_motor = null;

    @Override
    public void init() {

        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_back_motor.setPower(1);

    }else{
            left_back_motor.setPower(0);
        }
}

}
