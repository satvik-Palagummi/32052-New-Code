package org.firstinspires.ftc.teamcode.Sensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Balls {
    private int[] motif;
    private final int[] twentyOne = {0,1,1};
    private final int[] twentyTwo = {1,0,1};
    private final int[] twentyThree = {1,1,0};
    private int[] currentBalls;

    public void setMotif(int id){
        if(id == 21) {
            motif = twentyOne;
        }else if(id == 22){
            motif = twentyTwo;
        }else if(id == 23){
            motif = twentyThree;
        }
    }
    public int[] sortBalls(){
        int[] sorted = new int[3];
        int[] giveUpArray = {0,1,2};
        boolean giveUp = false;
        for(int i = 0; i<3;i++){
            int currentPos;
            if(!giveUp) {
                for (int j = 0; j < 3; j++) {
                    if (getMotif(i) == currentBalls[j]){
                        currentPos = j;
                        sorted[i] = currentPos;
                        currentBalls[currentPos] = -1;
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
    public void setCurrent(int num){
        if(num == 0){
            currentBalls = new int[]{-1, -1, -1};
        }
        if(num == 1){
            currentBalls = twentyThree;
        }
        if(num == 2){
            currentBalls = twentyTwo;
        }
        if(num == 3){
            currentBalls = twentyOne;
        }
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
