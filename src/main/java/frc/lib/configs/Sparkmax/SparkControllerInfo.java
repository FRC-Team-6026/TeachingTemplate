package frc.lib.configs.Sparkmax;

import com.revrobotics.CANSparkMax.IdleMode;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants.*;

public class SparkControllerInfo {
    public Usage canbusUse;
    public int currentLim;
    public boolean invert;
    public IdleMode idleMode;
    public double posConversion;
    public double velConversion;
    public double[] pidList;
    public double voltageComp;

    public SparkControllerInfo drive(){
        canbusUse = Usages.driveUsage;
        currentLim = Electical.driveCurrentLim;
        invert = Setup.driveInvert;
        idleMode = IdleModes.driveIdle;
        posConversion = ConversionFactors.driveConversionPositionFactor;
        velConversion = ConversionFactors.driveConversionVelocityFactor;
        pidList = PID.drivePID;
        voltageComp = Electical.voltageComp;
        return this;
    }

    public SparkControllerInfo angle(){
        canbusUse = Usages.angleUsage;
        currentLim = Electical.angleCurrentLim;
        invert = Setup.angleInvert;
        idleMode = IdleModes.angleIdle;
        posConversion = ConversionFactors.angleConversionPositionFactor;
        velConversion = ConversionFactors.angleConversionVelocityFactor;
        pidList = PID.anglePID;
        voltageComp = Electical.voltageComp;
        return this;
    }

    public SparkControllerInfo rotation(){
        canbusUse = Usages.rotationUsage;
        currentLim = Electical.rotationCurrentLim;
        invert = Setup.rotationInvert;
        idleMode = IdleModes.rotationIdle;
        posConversion = ConversionFactors.rotationConversionPositionFactor;
        velConversion = ConversionFactors.rotationConversionVelocityFactor;
        pidList = PID.rotationPID;
        voltageComp = Electical.voltageComp;
        return this;
    }

    public SparkControllerInfo extension(){
        canbusUse = Usages.extensionUsage;
        currentLim = Electical.extensionCurrentLim;
        invert = Setup.extensionInvert;
        idleMode = IdleModes.extensionIdle;
        posConversion = ConversionFactors.extensionConversionPositionFactor;
        velConversion = ConversionFactors.extensionConversionVelocityFactor;
        pidList = PID.extensionPID;
        voltageComp = Electical.voltageComp;
        return this;
    }


}
