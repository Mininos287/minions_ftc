package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "TEST_PID",group = "test")


public class TestPID extends LinearOpMode {

    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;

    double move_power=0 ;
    double  gyro_angel ;


    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    HelperClass helper_class_object = new HelperClass();


    @Override
    public void runOpMode() throws InterruptedException {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

        left_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();


        waitForStart();

        while (opModeIsActive()){
if(gamepad1.right_bumper){
    move_power = 75;
}else if(gamepad1.right_bumper && gamepad1.left_bumper){
    move_power = 100 ;

}else{
    move_power = 50 ;
}

            if (gamepad1.dpad_up){
                left_back_motor.setDirection(DcMotor.Direction.FORWARD);
                left_front_motor.setDirection(DcMotor.Direction.FORWARD);
                right_back_motor.setDirection(DcMotor.Direction.REVERSE);
                right_front_motor.setDirection(DcMotor.Direction.REVERSE);

                move(left_back_motor,left_front_motor,right_back_motor,right_front_motor,200,move_power);
            }
        }

    }

    public void move(DcMotor left_back_motor, DcMotor left_front_motor, DcMotor right_back_motor, DcMotor right_front_motor
            , double distance , double power ) {

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        double gyro_start = gyro_angel ;
        double left_power ;
        double right_power ;


        left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
        left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
        right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
        right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));

        int target_ticks = left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);


        double error = 0 ;
        double last_error = 0;
        double KP = 5;
        double KI = 0;
        double KD = 0;
        double probational=0 ;
        double derivative=0 ;
        double integral=0 ;

        while ((left_back_motor.getCurrentPosition() < target_ticks) && (left_front_motor.getCurrentPosition() < target_ticks)
                && (right_back_motor.getCurrentPosition() < target_ticks) && (right_front_motor.getCurrentPosition() < target_ticks))
        {
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            error= gyro_angel - gyro_start;

            probational = error ;
            integral = integral + error ;
            derivative = error - last_error;

            left_power = power + ( (probational * KP ) + (integral*KI) + (derivative*KD) ) ;
            right_power = power - ( (probational * KP ) + (integral*KI) + (derivative*KD) ) ;




            left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power),-100,100));
            left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power),-100,100));
            right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power),-100,100));
            right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power),-100,100));

            sleep(1);

            last_error = error ;

            telemetry.addData("LEFT_POWER and RIGHT_POWER","%f %f",left_power,right_power);
            telemetry.update();
        }


        left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
        left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
        right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
        right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));

        sleep(1);



    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        gyro_angel = Double.parseDouble(helper_class_object.formatAngle(angles.angleUnit, angles.firstAngle));
                        return helper_class_object.formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return helper_class_object.formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return helper_class_object.formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
}
