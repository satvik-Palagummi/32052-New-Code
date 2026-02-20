package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Sensor.Limelight;

import java.util.List;

@Autonomous
public class AprilTagLimelightTest extends OpMode {
    Limelight limelight = new Limelight();
    LLResult llResult;
    private boolean motifDetected;
    @Override
    public void init() {
        limelight.initLimelight(hardwareMap);
        limelight.setPipeline(8);
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
    public void scan(){
        llResult = limelight.updateLimelight();
        limelight.scanGoal();
    }

    @Override
    public void loop() {
        scan();
        if(llResult != null && llResult.isValid()){
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Distance in CM", limelight.getDistance(llResult.getTa()));
        }
    }
}
