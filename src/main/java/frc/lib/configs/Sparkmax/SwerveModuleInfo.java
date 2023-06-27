package frc.lib.configs.Sparkmax;

import com.ctre.phoenix.sensors.CANCoder;
import frc.lib.Items.SparkMax.SparkController;
import frc.robot.Constants;

public class SwerveModuleInfo {
    public int moduleNumber;
    public SparkController drive;
    public SparkController angle;
    public CANCoder cancoder;
    public double angleOffset;
    public double xPos;

    /**Requires the module to assign cancodes correctly
     * @param moduleNumber
     */

    public SwerveModuleInfo(int moduleNumber){
        this.moduleNumber = moduleNumber;
        drive = new SparkController(Constants.Setup.driveMotors[moduleNumber], new SparkControllerInfo().drive());
        angle = new SparkController(Constants.Setup.angleMotors[moduleNumber], new SparkControllerInfo().angle());
        cancoder = new CANCoder(Constants.Setup.moduleCancoders[moduleNumber]);
        angleOffset = Constants.Setup.angleOffsets[moduleNumber];
        xPos = Constants.Setup.xposition[moduleNumber];
    }
}
