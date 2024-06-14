package org.firstinspires.ftc.teamcode;

import java.util.*;

class Bezier extends Actions {
  private List<Pos2D> pts;

  private int numPts;
  double length;

  public Bezier(List<Pos2D> points) {
    pts = points;
    numPts = pts.size();
    length = (1/8)*(fOfX(0) + 3*fOfX(1/3) + 3*fOfX(2/3) + fOfX(1));
  }
  private Pos2D deriv(double t) {
    int n = pts.size() - 1;
    ArrayList<Pos2D> derivativePoints = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      double x = n * (pts.get(i + 1).x - pts.get(i).x) * Math.pow(1 - t, n - 1);
      double y = n * (pts.get(i + 1).y - pts.get(i).y) * Math.pow(1 - t, n - 1);
      derivativePoints.add(new Pos2D(x, y));
    }
    t*=(pts.size()-1);
    return derivativePoints.get((int)t);
  }

  private double fOfX(double t) {
    t = (int)(t*(pts.size()-1));
    double dy = Math.pow(deriv(t).y, 2);
    double dx = Math.pow(deriv(t).x, 2);
    return Math.sqrt(dy+dx);
  }
  
  public Pos2D getPos(double t){
    double x =0.0;
    double y =0.0;
    double theta = (1-t)*pts.get(0).theta + t*pts.get(1).theta;
    for(int i = 0; i< numPts; i++){
      double temp = pascalTriangle(numPts,i)*Math.pow((1-t),numPts-1);
      temp *= Math.pow(t,i);
      x += temp*pts.get(i).x;
      y += temp*pts.get(i).y;
    }
    return new Pos2D(x,y,theta);
  }
  private int pascalTriangle(int n, int k) {
    return (factorial(n))/(factorial(k)*factorial(n-k));
  }
  private int factorial(int n) {
    if (n == 1) {
      return 1;
    } else {
      return n * factorial (n-1);
    }
  }
  
}