package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import java.lang.Math;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;
    DcMotor linearSlide;
    DcMotor flyWheel;
    DcMotor armBase;
    CRServo claw;
    BNO055IMU imu;

    boolean driveMode = false;

    @Override
    public void init() {
        // Control Hub
        topLeft = hardwareMap.dcMotor.get("topLeft");           //1
        topRight = hardwareMap.dcMotor.get("topRight");         //0 
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft");     //2
        bottomRight = hardwareMap.dcMotor.get("bottomRight");   //3

        linearSlide = hardwareMap.dcMotor.get("linearSlide");   //4

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        // Expansion Hub
       /* claw = hardwareMap.crservo.get("claw");                 //0 CR Servo
        flyWheel = hardwareMap.dcMotor.get("flyWheel");         //0
        armBase = hardwareMap.dcMotor.get("armBase");           //1

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        armBase.setDirection(DcMotor.Direction.FORWARD);

        claw.setDirection(CRServo.Direction.FORWARD);

        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armBase.setPower(0);
        claw.setPower(0);
        flyWheel.setPower(0);*/
    }

    public void readEncoder(){
        telemetry.addData("armBase Encoder Ticks: ", armBase.getCurrentPosition());
        telemetry.update();
    }

    public double getAngle(){
        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void setDrivePow(double pow) {
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightY = gamepad1.right_stick_y;
        double rightX = gamepad1.right_stick_x;

        if (driveMode) {
            double A = getAngle(); // Orientation
            if (A < 0) {
                A = 360 + A;
            }

            double gamepadA = Math.atan2(leftY, leftX);
            gamepadA = 180/Math.PI * gamepadA;
            if (gamepadA < 0) {
                gamepadA = 360 + gamepadA;
            }
            gamepadA = 360 - gamepadA;
            gamepadA = (gamepadA + 270) % 360;
            if (Math.abs(leftX) < 0.1 && Math.abs(leftY) < 0.1) {
                gamepadA = 0;
            }

            double dA = A - gamepadA;
            if (dA < 0) {
                dA = 360 + dA;
            }

            dA = (dA + 90) % 360;

            double mag = Math.sqrt(leftX * leftX + leftY * leftY);
            double x = Math.cos(dA * Math.PI / 180) * mag;
            double y = Math.sin(dA * Math.PI / 180) * mag;

            topLeft.setPower(    (y - x + rightX) * pow);
            topRight.setPower(   (y + x - rightX) * pow);
            bottomLeft.setPower( (y + x + rightX) * pow);
            bottomRight.setPower((y - x - rightX) * pow);

        } else {
            topLeft.setPower(    (leftY - leftX - rightX) * -pow);
            topRight.setPower(   (leftY + leftX + rightX) * -pow);
            bottomLeft.setPower( (leftY + leftX - rightX) * -pow);
            bottomRight.setPower((leftY - leftX + rightX) * -pow);
        }
    }

    public void moveArm(int ticks, double power) {
        //armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setTargetPosition(ticks);
        armBase.setPower(power);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armBase.isBusy()) {
        }
        //armBase.setPower(0);
        //armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setArm(int ticks){
        int pos = armBase.getCurrentPosition();
        double err = ticks - pos;
        double pow = err / 2000;
        armBase.setPower(pow);
    }

    @Override
    public void loop()  {
        //readEncoder();

        if (gamepad1.x) { driveMode = true;  } //Absolute Drive
        if (gamepad1.b) { driveMode = false; } //Relative Drive

        //GamePad One - Drive Train
        if      (gamepad1.right_trigger > 0.6f) { setDrivePow(1);   } //Right Trigger = High Speed
        else if (gamepad1.left_trigger > 0.6f)  { setDrivePow(0.2); } //Left Trigger  = Low Speed
        else                                    { setDrivePow(0.5); } //No Trigger    = Normal Speed


        int slidePos = -linearSlide.getCurrentPosition();
        float slideTweenUp = 1;
        float slideTweenDown = 1;

        if (slidePos < 1000) {
            slideTweenDown = slidePos / 1000f;
        }
        if (slidePos > 3000) {
            slideTweenUp = (3500 - slidePos) / 500f;
        }

        linearSlide.setPower(0);
//        linearSlide.getCurrentPosition();
        if (gamepad1.y && slidePos < 3490f) { linearSlide.setPower(-0.99 * slideTweenUp); } //Up
        if (gamepad1.a && slidePos > 10f) { linearSlide.setPower( 0.99 * slideTweenDown); } //Down


        /*
        //GamePad Two - Arm
        armBase.setPower(-gamepad2.right_stick_y * 0.25);

        //GamePad Two - Fly Wheel
        if      (gamepad2.right_bumper) { flyWheel.setPower(1);  }
        else if (gamepad2.left_bumper)  { flyWheel.setPower(-1); }
        else                            { flyWheel.setPower(0);  }

        //GamePad Two - Claw
        double close = gamepad2.right_trigger;
        double open = gamepad2.left_trigger;

        if      (close > 0) { claw.setPower(close * -0.5); } //close
        else if (open > 0)  { claw.setPower(open *   0.5); } //open
        else                { claw.setPower(0);            } //stay

*/
        telemetry.addLine("Gamepad1 Feedback: ");
        telemetry.addLine("  gamepad1.right_trigger: " + gamepad1.right_trigger);
        telemetry.addLine("  gamepad1.left_trigger: " + gamepad1.left_trigger);
        telemetry.addLine("  gamepad1.right_bumper: " + gamepad1.right_bumper);
        telemetry.addLine("  gamepad1.left_bumper: " + gamepad1.left_bumper);
        telemetry.addLine("  gamepad1.left_stick_y: " + gamepad1.left_stick_y);
        telemetry.addLine("  gamepad1.left_stick_x: " + gamepad1.left_stick_x);
        telemetry.addLine("  gamepad1.right_stick_y: " + gamepad1.right_stick_y);
        telemetry.addLine("  gamepad1.right_stick_x: " + gamepad1.right_stick_x);
        if (driveMode) {
            telemetry.addLine("DRIVEMODE: ABSOLUTE");
        } else {
            telemetry.addLine("DRIVEMODE: RELATIVE");
        }

        telemetry.addLine("Linear Slide: " + linearSlide.getCurrentPosition());

        telemetry.update();

        // set to corresponding level depending on click
        // top level
        /*
        if          (gamepad2.y){
            claw.setPower(1);
            setArm(650);
        }
        // middle level
        else if     (gamepad2.b) {
            claw.setPower(1);
            setArm(230);
        }
        // bottom level
        else if    (gamepad2.a) {
            claw.setPower(1);
            setArm(135);
        } else if         (gamepad2.x) {
            claw.setPower(1);
            setArm(-30);
            claw.setPower(-1);
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            claw.setPower(0);
        }
        //default
        else {
            armBase.setPower(0);
        }


        if  (gamepad2.x) {
            claw.setPower(1);
            setArm(-60);
            claw.setPower(-1);
            try {
                Thread.sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            claw.setPower(0);
        }*/
    }
}
