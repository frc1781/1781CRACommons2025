package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.EEtimeOfFlight;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensation extends SubsystemBase
{
    //NOTE: DIGITAL INPUT RETURNS TRUE FOR BEAM NOT BROKEN AND FALSE FOR BROKEN BEAM
    //      SO IT REPORTING NEGATED VALUE FOR DETECTION OF SOMETHING 
    DigitalInput enterBeam;
    DigitalInput hopperBackBeam;
    DigitalInput hopperFrontBeam;
    DigitalInput exitBeam;
    private EEtimeOfFlight leftTOF;
    private EEtimeOfFlight rightTOF;
    private EEtimeOfFlight armTOF;
    private EEtimeOfFlight coralTimeOfFlight;
    private EEtimeOfFlight clawCoralTOF;

    public Sensation()
    {
        armTOF = new EEtimeOfFlight(Constants.SensationConstants.ARM_TOF_ID, 20);
        armTOF.tof.setRangeOfInterest(6, 6, 10, 10);
        leftTOF = new EEtimeOfFlight(Constants.SensationConstants.LEFT_FRONT_TOF_ID, 20);
        rightTOF = new EEtimeOfFlight(Constants.SensationConstants.RIGHT_FRONT_TOF_ID, 20);
        clawCoralTOF = new EEtimeOfFlight(Constants.SensationConstants.CLAW_CORAL_TOF_ID, 80);
        enterBeam = new DigitalInput(Constants.SensationConstants.enter);
        hopperBackBeam = new DigitalInput(Constants.SensationConstants.hopperBack);
        hopperFrontBeam = new DigitalInput(Constants.SensationConstants.hopperFront);
        exitBeam = new DigitalInput(Constants.SensationConstants.exit);
    }

    public void periodic() {
        Logger.recordOutput("Sensation/coralPresent", coralPresent());
        Logger.recordOutput("Sensation/coralInHopper", coralInHopper());
        Logger.recordOutput("Sensation/coralExitedHopper", coralExitedHopper());
        Logger.recordOutput("Sensation/armTOF", armTOF.getRange());
        Logger.recordOutput("Sensation/armTOFValid", armTOF.isRangeValidRegularCheck());
        Logger.recordOutput("Sensation/leftTOF", leftTOF.getRange());
        Logger.recordOutput("Sensation/leftTOFValid", leftTOF.isRangeValidRegularCheck());
        Logger.recordOutput("Sensation/rightTOF", rightTOF.getRange());
        Logger.recordOutput("Sensation/rightTOFValid", rightTOF.isRangeValidRegularCheck());
        Logger.recordOutput("Sensation/clawCoralTOF", clawCoralTOF.getRange());
        Logger.recordOutput("Sensation/clawCoralTOFValid", clawCoralTOF.isRangeValidRegularCheck());
    }

    public boolean coralPresent() {
        return !enterBeam.get() || !hopperBackBeam.get() || !hopperFrontBeam.get();
    }

    public boolean coralInHopper() {
        return !hopperBackBeam.get() || !hopperFrontBeam.get();
    }

    public boolean coralExitedHopper() {
        return !exitBeam.get() && hopperBackBeam.get() && hopperFrontBeam.get();
    }

    public double leftTOF(){
       return leftTOF.getRange();
    }

    public double rightTOF(){
        return rightTOF.getRange();
     }

     public double armTOF(){
        return armTOF.getRange();
     }

     public double coralTimeOfFlight(){
        return coralTimeOfFlight.getRange();
     }

     public boolean clawCoralPresent() {
         return clawCoralTOF.getRange() < 50 && clawCoralTOF.isRangeValidRegularCheck();
      }


     public boolean leftTOFisValid(){
        return leftTOF.isRangeValidRegularCheck();
     }
 
     public boolean rightTOFisValid(){
         return rightTOF.isRangeValidRegularCheck();
      }
 
      public boolean armTOFisValid(){
         return armTOF.isRangeValidRegularCheck();
      }
 
      public Boolean  coralTimeOfFlightisValid(){
         return coralTimeOfFlight.isRangeValidRegularCheck();
      }
    } 