package org.firstinspires.ftc.teamcode;

class Pos2D extends Actions {
  public double x;
  public double y;
  public double theta;

  public Pos2D(double xpos, double ypos, double t) {
    x = xpos;
    y = ypos;
    theta = t;
  }
  public Pos2D(double xpos, double ypos) {
    x = xpos;
    y = ypos;
    theta = 0;
  }
}