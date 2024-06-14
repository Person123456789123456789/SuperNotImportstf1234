package org.firstinspires.ftc.teamcode;

class Telemetry {
  Pos2D currentPos;

  public Telemetry(double x, double y, double theta) {
    currentPos = new Pos2D(x,y,theta);}
  public Telemetry(Pos2D startPos) {
    currentPos = startPos;}
  public Telemetry() {
    currentPos = new Pos2D(0,0,0);}

  public void updateTelemetry(double dLeft, double dRight, double dBack) {
    robotToFieldPos(dRobotPos(dLeft,dRight,dBack));
}

  private Pos2D dRobotPos(double dL, double dR, double dB) {
    double L,F,dtheta,dx,dy;
    L = DriveConstants.xWheelOffset;
    F = DriveConstants.yWheelOffset;
    dtheta = (dL-dR)/L;
    dx = (dL+dR)/2;
    dy = dB-(F*dtheta);
    return new Pos2D(dx,dy,dtheta);
  }

  private void robotToFieldPos(Pos2D dRobot) {
    double x = currentPos.x;
    double y = currentPos.y;
    double theta = currentPos.theta;
    double dx = dRobot.x;
    double dy = dRobot.y;
    double dtheta = dRobot.theta;
    x += dx*Math.cos(dtheta) - dy*Math.sin(dtheta);
    y += dx*Math.sin(dtheta) + dy*Math.cos(dtheta);
    theta += dtheta;
    currentPos = new Pos2D(x,y,theta);
  }
}