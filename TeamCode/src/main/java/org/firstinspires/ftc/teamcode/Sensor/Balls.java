package org.firstinspires.ftc.teamcode.Sensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

public class Balls {
    private int[] motif = {-1,-1,-1};
    private final int[] twentyOne = {0,1,1};
    private final int[] twentyTwo = {1,0,1};
    private final int[] twentyThree = {1,1,0};
    private int[] currentBalls = {-1,-1,-1};

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
        int[] giveUpArray = {0, 1, 2};
        if(!Arrays.equals(motif, new int[]{-1, -1, -1})) {
            boolean giveUp = false;
            boolean firstSort = false;
            for (int i = 0; i < 3; i++) {
                int currentPos;
                if (!giveUp) {
                    if (!firstSort){
                        for (int j = 0; j < 3; j++) {
                            if (getMotif(i) == currentBalls[j]) {
                                currentPos = j;
                                sorted[i] = currentPos;
                                currentBalls[currentPos] = -1;
                                firstSort = true;
                                break;
                            } else {
                                if (j == 2) {
                                    giveUp = true;
                                    break;
                                }
                            }
                        }
                    }else{
                        if(sorted[0] == 2){
                            for (int j = 1; j > -1; j--) {
                                if (getMotif(i) == currentBalls[j]) {
                                    currentPos = j;
                                    sorted[i] = currentPos;
                                    currentBalls[currentPos] = -1;
                                    break;
                                }/*else{
                                    if(j==0){
                                        giveUp = true;
                                        break;
                                    }
                                }
                                */
                            }
                        }else if(sorted[0]==0){
                            for (int j = 0; j < 2; j++) {
                                if (getMotif(i) == currentBalls[j]) {
                                    currentPos = j;
                                    sorted[i] = currentPos;
                                    currentBalls[currentPos] = -1;
                                    break;
                                }/*else{
                                    if(j==1){
                                        giveUp = true;
                                        break;
                                    }
                                }
                                */
                            }
                        }else{
                            for (int j = 0; j < 3; j++) {
                                if (getMotif(i) == currentBalls[j]) {
                                    currentPos = j;
                                    sorted[i] = currentPos;
                                    currentBalls[currentPos] = -1;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            if (giveUp) {
                return giveUpArray;
            }
            return sorted;
        }
        return giveUpArray;
    }
    public void setCurrent(int num){
        if(num == 1){
            currentBalls = new int[]{0,1,1};
        }
        if(num == 2){
            currentBalls = new int[]{1,0,1};
        }
        if(num == 3){
            currentBalls = new int[]{1,1,0};
        }
    }
    public void setCurrent(int[] num){
        currentBalls = num;
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
