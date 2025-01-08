package frc.robot.swerve.SwerveModule;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.swerve.DriveConstants;
import frc.robot.swerve.DriveConstants.SwerveModuleHardwareConfig;

public class SwerveModuleHardware {

  // Robot Specific info //
  private String name;
  private Rotation2d angleOffset;

  // Hardware //
  private TalonFX azimuth;
  private TalonFX drive;
  private CANcoder cancoder;

  // Status Signals //
  private StatusSignal<AngularVelocity> driveVelocity;
  private StatusSignal<Voltage> driveVoltage;
  private StatusSignal<Angle> drivePosistion;
  private StatusSignal<Current> driveStatorCurrent;
  private StatusSignal<Current> driveSupplyCurrent;
  private StatusSignal<Temperature> driveTempC;

  private StatusSignal<Angle> azimuthPosistion;
  private StatusSignal<Voltage> azimuthVoltage;
  private StatusSignal<Current> azimuthStatorCurrent;
  private StatusSignal<Current> azimuthSupplyCurrent;
  private StatusSignal<Temperature> azimuthTempC;

  private StatusSignal<Angle> absolutePosistionSignal;

  // Control //
  private VoltageOut azimuthVoltageControl;
  private VoltageOut driveVoltageControl;
  private PositionVoltage azimuthVoltagePosistion;
  private VelocityVoltage driveVelocityControl;

  // AutLog these values //
  @AutoLog
  public static class SwerveModuleInputs {
    public boolean driveConnected = true;
    public double driveVelocityMPS = 0.0;
    public double drivePosistionM = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveMotorVolts = 0.0;
    public double[] driveStator = {0.0};
    public double[] driveSupply = {0.0};
    public double[] driveTempC = {0.0};

    public boolean azimuthConnected = true;
    public Rotation2d azimuthPosistion = new Rotation2d();
    public Rotation2d azimuthAbsolutePosistion = new Rotation2d();
    public double azimuthAppliedVolts = 0.0;
    public double azimuthMotorVolts = 0.0;
    public double[] azimuthStator = {0.0};
    public double[] azimuthSupply = {0.0};
    public double[] azimuthTempC = {0.0};

  }

  public SwerveModuleHardware(SwerveModuleHardwareConfig modConstants){
    name = modConstants.name();
    angleOffset = modConstants.offset();

    drive = new TalonFX(modConstants.drivePort(), Constants.kCanbusName);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    // Ensure factory reset //
    drive.getConfigurator().apply(driveConfig);

    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 80;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60;
    driveConfig.MotorOutput.Inverted =InvertedValue.Clockwise_Positive; 
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Voltage.PeakForwardVoltage = 12.0;
    driveConfig.Voltage.PeakReverseVoltage = -12.0;
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveGearRatio / DriveConstants.circumfrenceMeters;
    driveConfig.Slot0.kP = DriveConstants.driveControllerConfig.kP();
    driveConfig.Slot0.kD = DriveConstants.driveControllerConfig.kD();
    // Apply new configs //
    drive.getConfigurator().apply(driveConfig);
  
    driveVelocity = drive.getVelocity();
    drivePosistion = drive.getPosition();
    driveVoltage = drive.getMotorVoltage();
    driveStatorCurrent = drive.getStatorCurrent();
    driveSupplyCurrent = drive.getSupplyCurrent();
    driveTempC = drive.getDeviceTemp();

    cancoder = new CANcoder(modConstants.cancoderPort(), Constants.kCanbusName);
    absolutePosistionSignal = cancoder.getAbsolutePosition();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    cancoder.getConfigurator().apply(encoderConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePosistionSignal);
    cancoder.optimizeBusUtilization();

    azimuth = new TalonFX(modConstants.azimuthPort(), Constants.kCanbusName);
    TalonFXConfiguration azimuthConfig = new TalonFXConfiguration();
    // Ensure factory reset //
    azimuth.getConfigurator().apply(azimuthConfig);

    azimuthConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    azimuthConfig.CurrentLimits.StatorCurrentLimit = 40;
    azimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    azimuthConfig.CurrentLimits.SupplyCurrentLimit = 30;
    azimuthConfig.MotorOutput.Inverted = (DriveConstants.invertAzimuths) ? 
    InvertedValue.Clockwise_Positive : 
    InvertedValue.CounterClockwise_Positive;
    azimuthConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    azimuthConfig.Voltage.PeakForwardVoltage = 12.0;
    azimuthConfig.Voltage.PeakReverseVoltage = -12.0;
    azimuthConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    azimuthConfig.Feedback.SensorToMechanismRatio = DriveConstants.azimuthGearRatio;
    azimuthConfig.Slot0.kP = DriveConstants.azimuthControllerConfig.kP();
    azimuthConfig.Slot0.kD = DriveConstants.azimuthControllerConfig.kD();
    azimuthConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // Apply new configs //
    azimuth.getConfigurator().apply(azimuthConfig);

    azimuthPosistion = azimuth.getPosition();
    azimuthVoltage = azimuth.getMotorVoltage();
    azimuthStatorCurrent = azimuth.getStatorCurrent();
    azimuthSupplyCurrent = azimuth.getSupplyCurrent();
    azimuthTempC = azimuth.getDeviceTemp();

    // Voltage control //
    azimuthVoltageControl = new VoltageOut(0);
    driveVoltageControl = new VoltageOut(0);

    // Closed - Loop control //
    azimuthVoltagePosistion = new PositionVoltage(0);
    driveVelocityControl = new VelocityVoltage(0);

    resetAzimuthPosistion();
  }

  public void updateInputs(SwerveModuleInputs inputs){
    inputs.driveConnected = BaseStatusSignal.refreshAll(
      driveVelocity,
      driveVoltage,
      driveStatorCurrent,
      driveSupplyCurrent,
      driveTempC).isOK();

    inputs.driveVelocityMPS = (driveVelocity.getValueAsDouble());
    inputs.drivePosistionM = drivePosistion.getValueAsDouble();
    inputs.driveMotorVolts = driveVoltage.getValueAsDouble();
    inputs.driveStator = new double[] {driveStatorCurrent.getValueAsDouble()};
    inputs.driveSupply = new double[] {driveSupplyCurrent.getValueAsDouble()};
    inputs.driveTempC = new double[] {driveTempC.getValueAsDouble()};

    inputs.azimuthConnected = BaseStatusSignal.refreshAll(
      absolutePosistionSignal,
      azimuthPosistion,
      azimuthVoltage,
      azimuthStatorCurrent,
      azimuthSupplyCurrent,
      azimuthTempC).isOK();

    inputs.azimuthPosistion = Rotation2d.fromRotations(azimuthPosistion.getValueAsDouble());
    inputs.azimuthAbsolutePosistion = Rotation2d.fromRotations(absolutePosistionSignal.getValueAsDouble()).minus((angleOffset));
    inputs.azimuthMotorVolts = azimuthVoltage.getValueAsDouble();
    inputs.azimuthStator = new double[] {azimuthStatorCurrent.getValueAsDouble()};
    inputs.azimuthSupply = new double[] {azimuthSupplyCurrent.getValueAsDouble()};
    inputs.azimuthTempC = new double[] {azimuthTempC.getValueAsDouble()};

  }

  public String getName(){
    return name;
  }

  public void setAzimuthVolts(double volts) {
    azimuth.setControl(azimuthVoltageControl.withOutput(volts));
  }

  public void setAzimuthPosistion(double posistion) {
    azimuth.setControl(azimuthVoltagePosistion.withPosition(posistion).withSlot(0));
  }

  public void setAzimuthConstants(double kP, double kD, double kS){
    Slot0Configs configs = new Slot0Configs();
    configs.kP = kP;
    configs.kD = kD;
    configs.kS = kS;
    azimuth.getConfigurator().apply(configs);
  }

  public void setDriveVolts(double volts) {
    drive.setControl(driveVoltageControl.withOutput(volts));
  }

  public void setDriveVelocity(double velocityMPS, double feedforward) {
    drive.setControl(driveVelocityControl.withVelocity(velocityMPS).withFeedForward(feedforward).withSlot(0));
  }

  public void setDriveConstants(double kP, double kD, double kS, double kV, double kA){
    Slot0Configs configs = new Slot0Configs();
    configs.kP = kP;
    configs.kD = kD;
    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    drive.getConfigurator().apply(configs);
  }

  public void resetAzimuthPosistion() {
    azimuth.setPosition(
      Rotation2d.fromRotations(
      absolutePosistionSignal.getValueAsDouble()).minus(angleOffset).getRotations(), 
      2.5);
  }
}
