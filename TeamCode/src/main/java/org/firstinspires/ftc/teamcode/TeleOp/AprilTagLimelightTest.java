package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

@Autonomous
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private boolean motifDetected;
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.setPollRateHz(30);
    }
    @Override
    public void start(){
        limelight.start();
    }
    public int scanMotif(LLResult result){
        if(result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (int ID = 21; ID <= 23; ID++) {
                if (!motifDetected) {
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        if (fr.getFiducialId() == ID) {
                            motifDetected = true;
                            return ID;
                        }
                    }
                }
            }
        }
        return -1;
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            telemetry.addData("Motif ID", scanMotif(llResult));
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        }
    }
}
