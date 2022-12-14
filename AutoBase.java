package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.CompVision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

//Most methods here are outdated and will not work
public abstract class AutoBase extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();

    public CompVision cam;

    DcMotor topLeft;
    DcMotor topRight;
    DcMotor bottomLeft;
    DcMotor bottomRight;

    DcMotor flyWheel;
    DcMotor armBase;

    CRServo claw;

    BNO055IMU imu;


    public void initRobo() {
        //Control Hub
        topLeft = hardwareMap.dcMotor.get("topLeft");           //1 Motor
        topRight = hardwareMap.dcMotor.get("topRight");         //0 Motor
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft");     //2 Motor
        bottomRight = hardwareMap.dcMotor.get("bottomRight");   //3 Motor

        topLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);

//        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

//        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Expansion Hub
//        claw = hardwareMap.crservo.get("claw");                 //1 CR Servo
//        flyWheel = hardwareMap.dcMotor.get("flyWheel");         //0 Motor
//        armBase = hardwareMap.dcMotor.get("armBase");           //1 Motor

//        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        flyWheel.setDirection(DcMotor.Direction.FORWARD);
 //       armBase.setDirection(DcMotor.Direction.FORWARD);

//        claw.setDirection(CRServo.Direction.FORWARD);

//        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

 //       armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
 //       flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //armBase.setPower(0);
        //claw.setPower(0);
        //flyWheel.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // experimental PID control - setup --------------------------
        //init orientation (old)
//        lastAngles = new Orientation();
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        globalAngle = 0;
    }


    public void move(double vx, double vy, double vtur, double pow) throws InterruptedException {
        double A = getAngle(); // Orientation
        if (A < 0) {
            A = 360 + A;
        }

        double desired = Math.atan2(vy, vx);
        desired = 180/Math.PI * desired;
        if (desired < 0) {
            desired = 360 + desired;
        }
        desired = 360 - desired;
        desired = (desired + 270) % 360;

        double dA = A - desired;
        if (dA < 0) {
            dA = 360 + dA;
        }

        dA = (dA + 90) % 360;

        double x = Math.cos(dA * Math.PI / 180);
        double y = Math.sin(dA * Math.PI / 180);

        topLeft.setPower(    (y - x + vtur) * pow);
        topRight.setPower(   (y + x - vtur) * pow);
        bottomLeft.setPower( (y + x + vtur) * pow);
        bottomRight.setPower((y - x - vtur) * pow);
    }






    public void moveArmBase(double power, int time) throws InterruptedException{
        armBase.setPower(power);
        Thread.sleep(time);
        armBase.setPower(0);
    }

    public void moveClaw(double power) throws InterruptedException{
        claw.setPower(power);
    }

    public void resetClaw(int time) throws InterruptedException {
        claw.setPower(0);
        claw.setPower(-1);
        Thread.sleep(time);
        claw.setPower(0);
    }
    public void closeClaw() throws InterruptedException {
        claw.setPower(1);
    }


    //experimental code with PID control ----------------------------
    public double getAngle(){
        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }


    public void setColumnPow(double powLeft, double powRight){
        topLeft.setPower(powLeft);
        topRight.setPower(powRight);
        bottomLeft.setPower(powLeft);
        bottomRight.setPower(powRight);
    }

    ///Set Single Power
    public void setSinglePow(double pow){
        topLeft.setPower(pow);
        topRight.setPower(pow);
        bottomLeft.setPower(pow);
        bottomRight.setPower(pow);
    }

    //Set Each Power
    public void setEachPow(double tLpow, double tRpow, double bLpow, double bRpow){
        topLeft.setPower(tLpow);
        topRight.setPower(tRpow);
        bottomLeft.setPower(bLpow);
        bottomRight.setPower(bRpow);
    }

    public void startFlywheel(double pow, int time) throws InterruptedException {
        flyWheel.setPower(pow);

        Thread.sleep(time);
        flyWheel.setPower(0);
    }

    //Read Motors encoders
    public void readEncoder(){
        telemetry.addData("topLeft Encoder Ticks: ", topLeft.getCurrentPosition());
        telemetry.addData("topRight Encoder Ticks: ", topRight.getCurrentPosition());
        telemetry.addData("bottomLeft Encoder Ticks: ", bottomLeft.getCurrentPosition());
        telemetry.addData("bottomRight Encoder Ticks: ", bottomRight.getCurrentPosition());
        telemetry.update();
    }

    //Read Arm encoders
    public void readEncoderArm(){
        telemetry.addData("Encoder Ticks: ", armBase.getCurrentPosition());
        telemetry.update();
    }
    public void moveArm(int ticks, double power) throws InterruptedException{
        //armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setTargetPosition(ticks);
        armBase.setPower(power);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armBase.isBusy()) {
            readEncoderArm();
        }
        //armBase.setPower(0);
        //armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Sets all target positions
    public void setAllMTargets(int left, int right){
        topLeft.setTargetPosition(left);
        topRight.setTargetPosition(right);
        bottomLeft.setTargetPosition(left);
        bottomRight.setTargetPosition(right);
    }


    //Linear Encoder movement
    public void moveRobotSmooth(int left, int right, double power) throws InterruptedException{
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setAllMTargets(-left, -right);

        topLeft.setPower(power);
        topRight.setPower(power);
        bottomLeft.setPower(power);
        bottomRight.setPower(power);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(topLeft.isBusy() || topRight.isBusy() || bottomLeft.isBusy() || bottomRight.isBusy()) {
            power = Math.max(power * 0.99, 0.1);
            topLeft.setPower(power);
            topRight.setPower(power);
            bottomLeft.setPower(power);
            bottomRight.setPower(power);
        }
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveRobot(int left, int right, double power) throws InterruptedException{
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setAllMTargets(-left, -right);

        topLeft.setPower(power);
        topRight.setPower(power);
        bottomLeft.setPower(power);
        bottomRight.setPower(power);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(topLeft.isBusy() || topRight.isBusy() || bottomLeft.isBusy() || bottomRight.isBusy()) {

        }
        topLeft.setPower(0);
        topRight.setPower(0);
        bottomLeft.setPower(0);
        bottomRight.setPower(0);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void linearY(double inches, double power) throws InterruptedException {
       // int ticks = (int)(inches/CIRCUMFERENCE*TPR);
        //moveRobot(ticks, ticks, power);
    }

    public void turnHeading(double angle) throws InterruptedException {
        turnHeading(angle,0.5f);
    }

    public void turnHeading(double angle, double pow) throws InterruptedException {
        double power = pow;
        double m_P = 5.5;
        double tol = 2.5;
        //double pow = 1;

        double err = (angle-this.getAngle());


        while(opModeIsActive() && Math.abs(err)>tol){
            int ticks = (int)(m_P*err);
            moveRobot(-ticks, ticks, power);
            err = (angle-this.getAngle());
            telemetry.addLine(String.valueOf(this.getAngle()));
            telemetry.update();
        }
    }
    public void turnHeadingSmooth(double angle, double pow) throws InterruptedException {
        double power = pow;
        double m_P = 5.5;
        double tol = 2.5;

        double err = (angle-getAngle());

        while(opModeIsActive() && Math.abs(err)>tol){
            int ticks = (int)(m_P*err);
            double sign = err / Math.abs(err);
            this.setColumnPow(-sign*pow, sign*pow);
            err = (angle-getAngle());
            telemetry.addLine(String.valueOf(this.getAngle()));
            telemetry.update();
        }
    }
}
