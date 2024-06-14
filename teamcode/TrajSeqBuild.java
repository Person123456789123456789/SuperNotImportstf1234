package org.firstinspires.ftc.teamcode;

import java.util.*;
class TrajSeqBuild {
  private List<Actions> trajSeq;
  private int ptsPerCm = 5;
  
  public TrajSeqBuild(List<Actions> curves) {
    trajSeq = new ArrayList<Actions>();
    for(int i = 0; i < curves.size(); i++) {
      if(curves.get(i) instanceof Bezier) {
        Bezier b = (Bezier)(curves.get(i));
        int numPts = (int)(b.length * ptsPerCm);
        for(int j = 0; j < numPts; j++) {
          trajSeq.add(b.getPos(j/numPts));
        }
      } else {
        trajSeq.add(curves.get(i));
      }
    }
    
  }
}