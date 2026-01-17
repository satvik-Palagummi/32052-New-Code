package org.firstinspires.ftc.teamcode.Sensor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight{
    public enum LimelightMode{
        GOAL_TAG,
        OBELISK_TAG
    }
    private LimelightMode currentMode;
    private Limelight3A limelight;
    private LLResult result;
    private boolean limelightIsOff;
    private boolean motifDetected;
    public static int detectedTagId;
    private double ta;
    public void initLimelight(HardwareMap hweM){
        limelight = hweM.get(Limelight3A.class, "Lemon Lamp");
        limelight.pipelineSwitch(8);
    }

    public void updateLimelight(){
        if(limelight != null){
            if(limelightIsOff){
                limelight.start();
                limelightIsOff = !limelightIsOff;
            }
            result = limelight.getLatestResult();
        }
    }
    public void scanMotif(){
        if(result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (int ID = 21; ID <= 23; ID++) {
                if (!motifDetected) {
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        if (fr.getFiducialId() == ID) {
                            limelight.stop();
                            detectedTagId = ID;
                            motifDetected = true;
                            limelightIsOff = true;
                            setLimelightMode(LimelightMode.GOAL_TAG);
                            break;
                        }
                    }
                }
            }
        }
    }
    public void scanGoal(){
        if(result != null && result.isValid()){
            setTa(result.getTa());
            limelight.stop();
            limelightIsOff = true;
        }
    }
    public void setTa(double ta){
        this.ta = ta;
    }
    public double getTa(){
        return ta;
    }
    public double getDistance(double ta){
        double scale = 30665.95;
        return (Math.pow((scale/ta),0.5));
    }
    public void setLimelightMode(LimelightMode newMode){
        currentMode = newMode;
    }



}
