package frc.lib.Items.SparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.lib.configs.Sparkmax.SparkControllerInfo;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class SparkController {
    public CANSparkMax spark;
    public RelativeEncoder sparkEncoder;
    public SparkMaxPIDController sparkControl;
    public final int canbusNumber;
    private final Usage canbusUse;
    private final int currentLim;
    private final boolean invert;
    private final IdleMode idleMode;
    private final double posConversion;
    private final double velConversion;
    private final double[] pidList;
    private boolean autoPid = false;
    private double[] pidAutoList;
    private final double voltageComp;
    private double max = 1;
    private double min = -1;
    public double fLim = 0;
    public double bLim = 0;
    public boolean fEnable = false;
    public boolean bEnable = false;


    /* Creates and Configures the Sparkmax Controller*/
    public SparkController(int canbusNumber, SparkControllerInfo Info){
        this.canbusNumber = canbusNumber;
        this.canbusUse = Info.canbusUse;
        this.currentLim = Info.currentLim;
        this.invert = Info.invert;
        this.idleMode = Info.idleMode;
        this.posConversion = Info.posConversion;
        this.velConversion = Info.velConversion;
        this.pidList = Info.pidList;
        this.voltageComp = Info.voltageComp;
        spark = new CANSparkMax(canbusNumber, MotorType.kBrushless);
        sparkEncoder = spark.getEncoder();
        sparkControl = spark.getPIDController();
        configureSpark();
     }

    /* Creates and Configures the Sparkmax Controller Note: Pass null to N/A fields */
    public SparkController(int canbusNumber, SparkControllerInfo Info, Double min, Double max, Double fLim, Double bLim){
    this.canbusNumber = canbusNumber;
    this.canbusUse = Info.canbusUse;
    this.currentLim = Info.currentLim;
    this.invert = Info.invert;
    this.idleMode = Info.idleMode;
    this.posConversion = Info.posConversion;
    this.velConversion = Info.velConversion;
    this.pidList = Info.pidList;
    this.voltageComp = Info.voltageComp;
    
    if(max != null){
        this.max = max;
    }
    if(min != null){
        this.min = min;
    }
    if(fLim != null){
        this.fLim = fLim;
        fEnable = true;
    }
    if(bLim != null){
        this.bLim = bLim;
        bEnable = true;
    }

    spark = new CANSparkMax(canbusNumber, MotorType.kBrushless);
    sparkEncoder = spark.getEncoder();
    sparkControl = spark.getPIDController();
    configureSpark();
    }

    public void addAutoPID(double[] pidAutoList){
        this.pidAutoList = pidAutoList;
        autoPid = true;
        configureSpark();
    }

    /* Sets and Flashes the Sparkmax to Passed States */
    public void configureSpark(){
        spark.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(spark, canbusUse);
        spark.setSmartCurrentLimit(currentLim);
        spark.setInverted(invert);
        spark.setIdleMode(idleMode);
        sparkEncoder.setVelocityConversionFactor(velConversion);
        sparkEncoder.setPositionConversionFactor(posConversion);
        sparkControl.setP(pidList[0], 0);
        sparkControl.setI(pidList[1], 0);
        sparkControl.setD(pidList[2], 0);
        sparkControl.setFF(pidList[3], 0);
        if(autoPid == true){
            sparkControl.setP(pidAutoList[0], 1);
            sparkControl.setI(pidAutoList[1], 1);
            sparkControl.setD(pidAutoList[2], 1);
            sparkControl.setFF(pidAutoList[3], 1);
        }
        spark.enableVoltageCompensation(voltageComp);
        sparkControl.setOutputRange(min, max);
        spark.setSoftLimit(SoftLimitDirection.kForward, ((float)fLim));
        spark.setSoftLimit(SoftLimitDirection.kReverse, ((float)bLim));
        spark.enableSoftLimit(SoftLimitDirection.kForward, fEnable);
        spark.enableSoftLimit(SoftLimitDirection.kReverse, bEnable);
        spark.burnFlash();
        sparkEncoder.setPosition(0.0);    
    }
    
}
