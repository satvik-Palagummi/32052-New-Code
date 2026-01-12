package org.firstinspires.ftc.teamcode.Sensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Balls {
    private ArrayList<Integer> motif;
    private final ArrayList<Integer> twentyOne = new ArrayList<>(List.of(0,1,1));
    private final ArrayList<Integer> twentyTwo = new ArrayList<>(List.of(1,0,1));
    private final ArrayList<Integer> twentyThree = new ArrayList<>(List.of(1,1,0));
    private int pos;
    public void setMotif(){
        if(Limelight.detectedTagId == 21) {
            motif = twentyOne;
        }else if(Limelight.detectedTagId == 22){
            motif = twentyTwo;
        }else if(Limelight.detectedTagId == 23){
            motif = twentyThree;
        }
    }
    public ArrayList<Integer> getFullMotif(){
        return motif;
    }
    public int getMotif(int pos){
        return motif.get(pos);
    }


}
