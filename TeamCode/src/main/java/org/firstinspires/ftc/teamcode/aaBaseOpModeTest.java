package org.firstinspires.ftc.teamcode;

import android.graphics.Rect;
import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
/*import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;*/

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public abstract class aaBaseOpModeTest extends LinearOpMode{

    ColorSensor colorSense;
    OpticalDistanceSensor rangeSense;
    Servo servo1;
    Servo servo2;
    Servo servo3;

    DcMotor motor1;
    DcMotor motor2;

    //Setting constant variables
    final double servoInitPosition = 0;

    // Trying to get range sensor to work: -Includes changing I2C address
    //DistanceSensor rangeSensor;
    /*byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;*/

    public void initilaize()
    {
        //Assigning previously declared variables to expansion hub names
        colorSense = hardwareMap.colorSensor.get("colorMR");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");
        rangeSense = hardwareMap.opticalDistanceSensor.get("rangeREV");

        motor2 = hardwareMap.dcMotor.get("motor2");
        motor1 = hardwareMap.dcMotor.get("motor1");

        //setting servo initial positions on initialize method
        servo1.setPosition(servoInitPosition);
        servo2.setPosition(servoInitPosition);
        servo3.setPosition(servoInitPosition);


        //rangeSensor = hardwareMap.get(DistanceSensor.class, "rangeREV");
        /*RANGE1 = hardwareMap.i2cDevice.get("rangeMR");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);*/

    }
}