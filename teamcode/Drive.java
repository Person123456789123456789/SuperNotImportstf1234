package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.DriveConstants.kV;

class Drive {
  public Drive(HardwareMap hardwareMap) {
    //super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
  }
  private DcMotorEx fL, fR, bL, bR;
  public static double LATERAL_MULTIPLIER = 3.5;
  private List<DcMotorEx> motors;
  private List<Integer> lastEncPositions = new ArrayList<>();
  private List<Integer> lastEncVels = new ArrayList<>();
  public void setVel(double fV,double sV,double tV){
    double R = DriveConstants.wheelTrackOffset;
    //change variables to actual motor powers
    fL.setVelocity(cmToTicksVel(fV - sV - (2 * R * tV)));
    fR.setVelocity(cmToTicksVel(fV + sV + (2 * R * tV)));
    bL.setVelocity(cmToTicksVel(fV + sV - (2 * R * tV)));
    bR.setVelocity(cmToTicksVel(fV - sV + (2 * R * tV)));
  }
  public WheelVelocities getVel() {
    double fLpower = fL.getVelocity();
    double fRpower = fR.getVelocity();
    double bLpower = bL.getVelocity();
    double bRpower = bR.getVelocity();
    return new WheelVelocities(fLpower, fRpower, bLpower, bRpower);
  }
  public double cmToTicksVel(double cm) {
    return (cm * DriveConstants.TICKS_PER_REV) / (Math.PI * Math.pow(DriveConstants.WHEEL_RADIUS, 2));
  }
  public double ticksToCmVel(double ticks) {
    return (ticks / DriveConstants.TICKS_PER_REV) * (Math.PI * Math.pow(DriveConstants.WHEEL_RADIUS, 2));
  }

  //public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(4, 0, 0);
  //public static PIDCoefficients HEADING_PID = new PIDCoefficients(9, 0, 0);


    follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
            new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

    LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

    batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    leftFront = hardwareMap.get(DcMotorEx.class, "BR");
    leftRear = hardwareMap.get(DcMotorEx.class, "FR");
    rightRear = hardwareMap.get(DcMotorEx.class, "FL");
    rightFront = hardwareMap.get(DcMotorEx.class, "BL");

    motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

    for (DcMotorEx motor : motors) {
      MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
      motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
      motor.setMotorType(motorConfigurationType);
    }

    setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

    List<Integer> lastTrackingEncPositions = new ArrayList<>();
    List<Integer> lastTrackingEncVels = new ArrayList<>();

    setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

    trajectorySequenceRunner = new TrajectorySequenceRunner(
            follower, HEADING_PID, batteryVoltageSensor,
            lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
    );
  }


  /*public List<Double> getWheelPositions() {
    lastEncPositions.clear();

    List<Double> wheelPositions = new ArrayList<>();
    for (DcMotorEx motor : motors) {
      int position = motor.getCurrentPosition();
      lastEncPositions.add(position);
      wheelPositions.add(encoderTicksToInches(position));
    }
    return wheelPositions;
  }

  trajectorySequenceRunner = new TrajectorySequenceRunner(
          follower, HEADING_PID, batteryVoltageSensor,
          lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
          );
}


waitForIdle();
  }


public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
  for (DcMotorEx motor : motors) {
    motor.setZeroPowerBehavior(zeroPowerBehavior);
  }
}*/

  public WheelVelocities getVel() {
    double fLpower.getVelocity();
    double fRpower; //change to motor powers
    double bLpower;
    double bRpower;
    return new WheelVelocities(fLpower, fRpower, bLpower, bRpower);
  }
}






















