package org.firstinspires.ftc.teamcode.Sensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Balls {
    private int[] currentBalls;
    private int[] motif;
    private final int[] twentyOne = new int[] {0,1,1};
    private final int[] twentyTwo = new int[] {1,0,1};
    private final int[] twentyThree = new int[] {1,1,0};

    public void setMotif(){
        if(Limelight.detectedTagId == 21) {
            motif = twentyOne;
        }else if(Limelight.detectedTagId == 22){
            motif = twentyTwo;
        }else if(Limelight.detectedTagId == 23){
            motif = twentyThree;
        }
    }
    public int[] sortBalls(){
        int[] sorted = new int[3];
        currentBalls = new int[3];
        int[] giveUpArray = {0,1,2};
        boolean giveUp = false;
        for(int i = 0; i<3;i++){
            int currentPos = 0;
            if(!giveUp) {
                for (int j = 0; j < 3; j++) {
                    if (getMotif(i) == currentBalls[j]){
                        currentPos = j;
                        sorted[i] = currentPos;
                        currentBalls[currentPos] = -1;
                        j=2;
                        break;
                    }else{
                        if(j==2){
                            giveUp = true;
                            break;
                        }
                    }
                }
            }
        }
        if(giveUp){
            return giveUpArray;
        }
        return sorted;
    }
    public void setCurrent(int[] current){
        currentBalls = current;
    }
    public int[] getCurrentBalls(){
        return currentBalls;
    }
    public int[] getFullMotif(){
        return motif;
    }
    public int getMotif(int pos){
        return motif[pos];
    }


}
