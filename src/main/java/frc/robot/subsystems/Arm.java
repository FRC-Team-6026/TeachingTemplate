package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Items.SparkMax.SparkController;
import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.robot.Constants;

public class Arm extends SubsystemBase{

    Solenoid grabber;
    SparkController rotation;
    SparkController extension;
    
    public Arm(){
        this.grabber = new Solenoid(Constants.Setup.grabberID, PneumaticsModuleType.REVPH, 1);
        this.rotation = new SparkController(Constants.Setup.rotationId , new SparkControllerInfo().rotation());
        this.extension = new SparkController(Constants.Setup.extensionId, new SparkControllerInfo().extension());
    }
    
}
