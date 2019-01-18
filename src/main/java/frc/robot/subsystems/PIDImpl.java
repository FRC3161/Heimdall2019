package frc.robot.subsystems;

import frc.robot.subsystems.TalonPIDSource;
import frc.robot.subsystems.drivetrain.OmniPod;
import frc.robot.subsystems.drivetrain.OmniPodImpl;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;

public class PIDImpl implements PID {

    PIDController pidcontrol;
    TalonPIDSource talonPID;
    PIDSource pidsource;
    PIDOutput pidoutput;
    
    double Kp;
    double Ki;
    double Kd; 

    public PIDImpl() {
        //TODO Do this for all encoders (currently only for FL, will rename later)
        this.Kp = 0.001;
        this.Ki = 0.001;
        this.Kd = 0.001;

        //this.talonPID = new TalonPIDSource();
        
        this.pidcontrol = new PIDController(Kp, Ki, Kd, talonPID, pidoutput);
    }


}
