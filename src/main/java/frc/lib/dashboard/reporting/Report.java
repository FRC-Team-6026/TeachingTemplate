package frc.lib.dashboard.reporting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.Sparkmax.SwerveModuleInfo;
import frc.robot.subsystems.SwerveModule;

/* This is classified as a SubsystemBase so it can Hyjack the WPILib default command, this can be changed by instead registering this as a class and calling the method in Robot.java 's robot periodic method.' */

public class Report extends SubsystemBase{
    private SwerveModule[] modules;

    /* Initialize the Required Items/Sensors Here */
    public Report(){
        for(int i = 0; i < 3; i++){
            modules[i] = new SwerveModule(new SwerveModuleInfo(i));
        }
    }

    /* All Applicable Data Parsing and Handling Should Go Here, However only sensor values can be reasonably found through this method and other subsystem variables should be reported through the default commands. */
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
