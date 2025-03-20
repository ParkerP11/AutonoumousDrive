package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;


/*

Parker Pruitt, 2025
FTC team Ubett 8672

A autonomous library for movement based on position
using the Goblida Pinpoint Co-Processor

The coordinate System is (0,0) is the left corner of your starting side
Each tile is 24 inches
X is vertical forward backward axis
Y is the horizontal side to side axis
Standard (x,y) -> Our coordinates are (y,x)

 */
public class AutonomousDrive {

    //Error Tolerances
    private final double POS_ERROR_TOLERANCE = 0.01;
    private final double HEADING_ERROR_TOLERANCE = 0.01;

    private final double MAX_MOTOR_CURRENT = 9.5;
    //PID controls

    //For Drive Movement
    private static double kDP = 1;
    private static double kDI = 0;
    private static double kDD = 0;
    private static double errorSumD = 0;
    private static double errorSumRangeD = 5;

    //For Turn Movement
    private static double kTP = 0.25;
    private static double kTI = 0.01;
    private static double kTD = 0.02;
    private static double errorSumT = 0;
    private static double errorSumRangeT = 25;



    //Drive motor names
    private String leftFrontName = "lf";
    private String leftBackName = "lb";
    private String rightFrontName = "rf";
    private String rightBackName = "rb";

    //Drive motor vars
    private static DcMotorEx lf;
    private static DcMotorEx lb;
    private static DcMotorEx rf;
    private static DcMotorEx rb;

    private static int leftFrontNum;
    private static int leftBackNum;
    private static int rightFrontNum;
    private static int rightBackNum;

    private static DcMotorControllerEx motorControllerEx;

    private static double[] motorCurrents = new double[4];

    //Gobilda Pinpoint
    private String odoName = "odo";
    private GoBildaPinpointDriver odo;

    //Pinpoint offsets from Center in mm and encoder Direction
    //Left is +X, Front is +Y
    private double xoffset = 100;
    private double yoffset = 100;

    private GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    //Dead Wheel Type

    private GoBildaPinpointDriver.GoBildaOdometryPods encoderPod = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    //LinearOpmode

    private LinearOpMode opMode;


    //Start Positions as {y,x} format and add to the master list for poses

    private double[] pos1 = new double[] {0,0, 0};
    private double[] pos2 = new double[] {8,36, 0 };


    private double[][] masterStartPoses = new double[][] { pos1, pos2};


    //Background Varables

    public static double timeLimit = 0;
    public static boolean outInfo = true;





    //Default Constructor
    public AutonomousDrive(LinearOpMode opMode, int startPos){
        this.opMode = opMode;

        lf = opMode.hardwareMap.get(DcMotorEx.class, leftFrontName);
        lb = opMode.hardwareMap.get(DcMotorEx.class, leftBackName);
        rf = opMode.hardwareMap.get(DcMotorEx.class, rightFrontName);
        rb = opMode.hardwareMap.get(DcMotorEx.class, rightBackName);

        leftFrontNum = lf.getPortNumber();
        leftBackNum = lb.getPortNumber();
        rightFrontNum = rf.getPortNumber();
        rightBackNum = rb.getPortNumber();

        motorControllerEx = (DcMotorControllerEx)(lf.getController());

        motorCurrents = getMotorCurrents();

        motorControllerEx.setMotorCurrentAlert(leftFrontNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(leftBackNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(rightFrontNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(rightBackNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);

        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, odoName);

        //Pinpoint offsets from Center in mm
        //Left is +X, Front is +Y
        odo.setOffsets(xoffset, yoffset);
        odo.setEncoderDirections(xDirection, yDirection);
        odo.setEncoderResolution(encoderPod);

        odo.recalibrateIMU();
        opMode.sleep(50);
        odo.resetPosAndIMU();
        opMode.sleep(50);

        odo.setPosition(new Pose2D(DistanceUnit.INCH, masterStartPoses[startPos][1],masterStartPoses[startPos][0], AngleUnit.DEGREES, masterStartPoses[startPos][0]));
        opMode.sleep(10);
    }


    //Getters and Setters and basic functions
    public static void drive(double rfPower, double rbPower, double lbPower, double lfPower) {

        rf.setPower(rfPower);
        rb.setPower(rbPower);
        lb.setPower(lbPower);
        lf.setPower(lfPower);
    }

    public GoBildaPinpointDriver getPinPoint(){return odo; }

    public Pose2D getPos(){return  odo.getPosition(); }
    public double getX(){return  odo.getPosition().getX(DistanceUnit.INCH); }
    public double getY(){return  odo.getPosition().getY(DistanceUnit.INCH); }

    public double getHeading(){
        double rawHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        return  rawHeading + 180;
    }

    public double getHeadingNorm(){
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public double getAngleToGo(double targetHeading){
        targetHeading = Math.abs(targetHeading) % 360;

        double currentHeading = getHeading();
        double angleTogo = targetHeading - currentHeading;
        currentHeading =  getHeading();

        angleTogo = targetHeading - currentHeading;

        if(Math.abs(angleTogo) > 180) {
            if (currentHeading < 180) {
                angleTogo = -((currentHeading) + (360 - targetHeading));
            } else {
                angleTogo = (targetHeading + (360 - currentHeading));
            }
        }
        return angleTogo;
    }

    public static double[] getMotorCurrents(){
        double currentLF = motorControllerEx.getMotorCurrent(leftFrontNum, CurrentUnit.AMPS);
        double currentLB = motorControllerEx.getMotorCurrent(leftBackNum, CurrentUnit.AMPS);
        double currentRF = motorControllerEx.getMotorCurrent(rightFrontNum, CurrentUnit.AMPS);
        double currentRB = motorControllerEx.getMotorCurrent(rightBackNum, CurrentUnit.AMPS);
        return new double[] {currentRF, currentRB, currentLB, currentLF};
    }

    public static ArrayList<Boolean> isMotorsOver(){
        boolean overCurrentLF = motorControllerEx.isMotorOverCurrent(leftFrontNum);
        boolean overCurrentLB = motorControllerEx.isMotorOverCurrent(leftBackNum);
        boolean overCurrentRF = motorControllerEx.isMotorOverCurrent(rightFrontNum);
        boolean overCurrentRB = motorControllerEx.isMotorOverCurrent(rightBackNum);

        ArrayList<Boolean> list = new ArrayList<>();

        list.add(overCurrentLF);
        list.add(overCurrentLB);
        list.add(overCurrentRF);
        list.add(overCurrentRB);

        return list;
    }

    public void setPID(double kP, double kI, double kD, int PIDNum){
        switch (PIDNum){
            case 0:
                kDP = kP;
                kDI = kI;
                kDD = kD;
                break;
            case 1:
                kTP = kP;
                kTI = kI;
                kTD = kD;
                break;

        }
    }


    //Set Limit to 0 if you don't want a time limit
    //Default is 0
    public static void setTimeLimit(double time){
        timeLimit = time;
    }

    public boolean checkTime(double startTime, double currentTime){
        if((currentTime - startTime) >= timeLimit){
            return true;
        }else {
            return false;
        }
    }

    public void setOutputInfo(boolean val){
        outInfo = val;
    }

    public void outputInfo(){
        if(outInfo){
            opMode.telemetry.addData("X pos: ", getX());
            opMode.telemetry.addData("Y pos: ", getY());
            opMode.telemetry.addData("Heading: ", getHeading());
            opMode.telemetry.addData("Heading Norm: ", getHeadingNorm());
            opMode.telemetry.addData("Heading UnNorm: ", odo.getHeading());
            opMode.telemetry.update();
        }
    }


    //PIDs

    private static double movePID(double error){
        double output = error * kDP - error * kDD + errorSumD*kDI;
        if(Math.abs(error) <= errorSumRangeD){
            errorSumD += error;
        }
        return output;
    }

    private static double turnPID(double error){
        double output = error * kTP - error * kTD + errorSumT*kTI;
        if(Math.abs(error) <= errorSumRangeT){
            errorSumT += error;
        }
        return output;
    }

    //Movement
    public void goToPointConstantHeading(double targetX, double targetY){
        odo.update();

        double targetXDist = targetX - getX();
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);


        double startHeading = getHeading();
        double turn = turnPID(startHeading);

        double startTime = opMode.time;


        while(!checkTime(startTime,opMode.time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE || Math.abs(getAngleToGo(startHeading)) > HEADING_ERROR_TOLERANCE)
                && opMode.opModeIsActive()) {

            odo.update();
            outputInfo();

            targetXDist = targetX - getX();
            targetYDist = targetY - getY();
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeading = getHeadingNorm();

            double angleToGo = getAngleToGo(startHeading);

            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = -movePID(targetXDist); // Remember, Y stick value is reversed
            double x = movePID(targetYDist);
            double rx = turnPID(angleToGo);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
            double rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            v1 = (rotY + rotX + rx) / denominator;
            v3 = (rotY - rotX + rx) / denominator;
            v2 = (rotY - rotX - rx) / denominator;
            v4 = (rotY + rotX - rx) / denominator;



            drive(v2,v4,v3,v1);
        }
        opMode.sleep(50);
    }

    public void goToPointLinear(double targetX, double targetY, double targetHeading){
        odo.update();

        double targetXDist = targetX - getX();
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);


        double startHeading = getHeading();
        double turn = turnPID(startHeading);

        double startTime = opMode.time;


        while(!checkTime(startTime,opMode.time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE || Math.abs(getAngleToGo(startHeading)) > HEADING_ERROR_TOLERANCE)
                && opMode.opModeIsActive()) {

            odo.update();
            outputInfo();

            targetXDist = targetX - getX();
            targetYDist = targetY - getY();
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeading = getHeadingNorm();

            double angleToGo = getAngleToGo(startHeading);

            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = -movePID(targetXDist); // Remember, Y stick value is reversed
            double x = movePID(targetYDist);
            double rx = turnPID(angleToGo);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
            double rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            v1 = (rotY + rotX + rx) / denominator;
            v3 = (rotY - rotX + rx) / denominator;
            v2 = (rotY - rotX - rx) / denominator;
            v4 = (rotY + rotX - rx) / denominator;



            drive(v2,v4,v3,v1);
        }
        opMode.sleep(50);
    }

    public void goToHeading(double heading){
        heading = Math.abs(heading) % 360;
        odo.update();
        double currentHeading = getHeading();
        double angleToGo = getAngleToGo(heading);
        double power;
        while(angleToGo > HEADING_ERROR_TOLERANCE){
            outputInfo();
            power = turnPID(angleToGo);
            drive(power, power,-power,-power);
        }
        opMode.sleep(50);
    }


}
