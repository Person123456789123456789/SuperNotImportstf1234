package org.firstinspires.ftc.teamcode;


public class DriveConstants {


  public static double xWheelOffset;
  public static double yWheelOffset;
  public static final double TICKS_PER_REV = 383.6;
  public static final double MAX_RPM = 435;
  public static double WHEEL_RADIUS = 4.8 ; // in
  public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
  public static double TRACK_WIDTH = 25.908; // cm
  public static double kV =0.013;
  public static double kA = 0.0025;
  public static double kStatic = 0.18;

  public static double encoderTicksToCm(double ticks) {
    return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
  }
  public static double wheelTrackOffset;
}


/*
 * Constants shared between multiple drive types.
 *
 * Constants generated by LearnRoadRunner.com/drive-constants
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */





  /*
   * These are the feedforward parameters used to model the drive motor behavior. If you are using
   * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
   * motor encoders or have elected not to use them for velocity control, these values should be
   * empirically tuned.
   */








