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
    public void initLimelight(HardwareMap hweM){
        limelight = hweM.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void updateLimelight(){
        if(limelight != null){
            result = limelight.getLatestResult();
        }
    }
    public void scanMotif(){
        if(result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (int ID = 21; ID <= 23; ID++) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == ID) {
                        limelight.stop();
                        detectedTagId = ID;
                        motifDetected = true;
                        limelightIsOff = true;
                        setLimelightMode(LimelightMode.GOAL_TAG);
                    }
                }
            }
        }
    }
    public void setLimelightMode(LimelightMode newMode){
        currentMode = newMode;
    }



}
