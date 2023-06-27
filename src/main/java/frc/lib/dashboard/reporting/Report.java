package frc.lib.dashboard.reporting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.configs.Sparkmax.SwerveModuleInfo;
import frc.robot.subsystems.SwerveModule;

public class Report{
    private SwerveModule[] modules;

    /* Initialize the Required Items Here */
    public Report(){
        for(int i = 0; i < 3; i++){
            modules[i] = new SwerveModule(new SwerveModuleInfo(i));
        }
    }

    /* All Data Parsing and Handling Should Go Here */
    public void report(){
        for (SwerveModule mod : modules) {
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);      
          }
    }

}
