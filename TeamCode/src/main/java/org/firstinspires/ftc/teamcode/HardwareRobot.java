package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * Created by fibonacci on 12/9/17.
 */

public class HardwareRobot {

    /* Public OpMode members. */
    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor rearRightMotor = null;
    public DcMotor rearLeftMotor = null;

    public DcMotor leftLiftMotor = null;
    public DcMotor rightLiftMotor = null;

    public Servo jewelPusherServo = null;

    public ColorSensor colorSensor = null;

    public DcMotor grabberMotor = null;

    public AutoMode autoMode = AutoMode.NULL;
    public AllianceColor allianceColor = AllianceColor.NULL;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    static final float lowerServoPosition = 0.65f;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        retrieveCustomAutonomousVariables();
        // Define and Initialize Motors
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        rearLeftMotor = hwMap.get(DcMotor.class, "rearLeftMotor");
        rearRightMotor = hwMap.get(DcMotor.class, "rearRightMotor");

        leftLiftMotor = hwMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hwMap.get(DcMotor.class, "rightLiftMotor");

        grabberMotor = hwMap.get(DcMotor.class, "grabberMotor");
        //End Initialize Motors

        jewelPusherServo = hwMap.get(Servo.class, "jewelServo");
        jewelPusherServo.setDirection(Servo.Direction.FORWARD);

        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        //Set Motor Directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        grabberMotor.setDirection(DcMotor.Direction.FORWARD);
        //End Set Motor Directions

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        leftLiftMotor.setPower(0);
        rightLiftMotor.setPower(0);

        grabberMotor.setPower(0);

        jewelPusherServo.setPosition(0);

        //End Power Set

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Retrieve values for custom values AllianceColor and AutoMode from FtcRobotControllerActivity
    public void retrieveCustomAutonomousVariables(){

        //Retrieve Alliance Color
        switch(FtcRobotControllerActivity.allianceColorString){

            case "RED":
                allianceColor = AllianceColor.RED;
                break;

            case "BLUE":
                allianceColor = AllianceColor.BLUE;
                break;

            case "NULL":
                allianceColor = AllianceColor.NULL;
                break;

        }

        //Retrieve Autonomous Mode
        switch(FtcRobotControllerActivity.autoModeString){

            case "IDOL_SIDE":
                autoMode = AutoMode.IDOL_SIDE;
                break;

            case "CRYPTOBOX_SIDE":
                autoMode = AutoMode.CRYPTOBOX_SIDE;
                break;

            case "NULL":
                autoMode = AutoMode.NULL;
                break;

        }
    }

    public void stopRobot(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

        grabberMotor.setPower(0);
        rightLiftMotor.setPower(0);
        leftLiftMotor.setPower(0);
    }

    //Positive = Opening, Negative = Closing
    public void setGrabberPower(float power){
        grabberMotor.setPower(power);
    }

    public void driveAtSpeed(float leftSpeed, float rightSpeed){
        frontLeftMotor.setPower(leftSpeed);
        rearLeftMotor.setPower(leftSpeed);
        frontRightMotor.setPower(rightSpeed);
        rearRightMotor.setPower(rightSpeed);

    }

    public void strafeRight(float strafeSpeed){
        frontRightMotor.setPower(strafeSpeed);
        rearRightMotor.setPower(-strafeSpeed);
        frontLeftMotor.setPower(-strafeSpeed);
        rearLeftMotor.setPower(strafeSpeed);
    }

    public void strafeLeft(float strafeSpeed){
        frontRightMotor.setPower(-strafeSpeed);
        rearRightMotor.setPower(strafeSpeed);
        frontLeftMotor.setPower(strafeSpeed);
        rearLeftMotor.setPower(-strafeSpeed);
    }

    public void dropServo(){
        jewelPusherServo.setPosition(lowerServoPosition);
    }

    public void raiseServo(){
        jewelPusherServo.setPosition(0);
    }

    public Color getProbableColor(){
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        int largestValue = Math.max(red, Math.max(green, blue));

        if(largestValue == red){
            return Color.RED;
        }else if(largestValue == green){
            return Color.GREEN;
        }else if(largestValue == blue){
            return Color.BLUE;
        }

        if(largestValue == 0){
            return Color.BLACK;
        }
        return Color.WHITE;
    }

    public boolean isAllainceColor(Color color){
        return colorsAreEqual(this.allianceColor, color);
    }

    public boolean colorsAreEqual(AllianceColor allianceColor, Color color){

        String colorString = color.toString();
        String allainceString = allianceColor.toString();

        if(colorString == allainceString){
            return true;
        }
        return false;
    }
}
