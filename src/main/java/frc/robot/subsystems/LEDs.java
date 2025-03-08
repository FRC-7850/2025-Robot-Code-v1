package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.lang.module.ModuleDescriptor.Requires;
import java.time.Year;
import java.util.Map;
import java.util.concurrent.CyclicBarrier;
import java.util.function.IntToDoubleFunction;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class LEDs extends SubsystemBase{
    ShuffleboardTab LEDTestingTab = Shuffleboard.getTab("LED Testing");
    GenericEntry LEDSP;
    double LEDSetpoint = 0;
    AddressableLED m_led = new AddressableLED(Constants.LEDConstants.LEDPWM);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.NUMLED);
    AddressableLEDBufferView m_underglow = m_ledBuffer.createView(Constants.LEDConstants.UNDERGLOWSTART, Constants.LEDConstants.UNDERGLOWEND);
    AddressableLEDBufferView m_elevator = m_ledBuffer.createView(Constants.LEDConstants.ELEVATORSTART, Constants.LEDConstants.ELEVATOREND);
    LEDPattern allianceColor = LEDPattern.solid(Color.kWhite).atBrightness(Percent.of(Constants.LEDConstants.BRIGHTPERCENT));
    //LEDPattern elevatorPattern = LEDPattern.gradient(GradientType.kContinuous,RGB2GRB(Color.kRed), RGB2GRB(Color.kWhite)).scrollAtRelativeSpeed(Percent.per(Second).of(25));
    LEDPattern elevatorPattern = LEDPattern.solid(RGB2GRB(Color.kRed));//.atBrightness(Percent.of(Constants.LEDConstants.BRIGHTPERCENT));

    Comparable<DriverStation.Alliance> allianceComparable = null;
    LEDPattern allOff = LEDPattern.solid(Color.kBlack);
    LEDPattern allWhite = LEDPattern.solid(RGB2GRB(Color.kWhite));
    ElevatorSubsystem m_elevatorSubsystem;
    public LEDs(ElevatorSubsystem Elevator){
        LEDSP = LEDTestingTab.add("SetPoint",0).getEntry();
        m_elevatorSubsystem = Elevator;
        m_led.setLength(m_ledBuffer.getLength());
        //allOff.applyTo(m_ledBuffer);
        //allianceColor = LEDPattern.solid(RGB2GRB(Color.kWhite)).atBrightness(Percent.of(Constants.LEDConstants.BRIGHTPERCENT));
        allianceColor.applyTo(m_underglow); 
        //setElevatorGradiant();
        m_led.setData(m_ledBuffer);
        
        
        m_led.start();


    }

    public void setUnderGlow(Comparable<DriverStation.Alliance> alliance){
        

        if(alliance != allianceComparable){
            if(alliance == Alliance.Blue){
                allianceColor = LEDPattern.solid(RGB2GRB(Color.kFirstBlue));//.atBrightness(Percent.of(Constants.LEDConstants.BRIGHTPERCENT));
            }
            else if(alliance == Alliance.Red){
                allianceColor = LEDPattern.solid(RGB2GRB(Color.kRed));//.atBrightness(Percent.of(Constants.LEDConstants.BRIGHTPERCENT));
            }
            else{
                allianceColor = LEDPattern.solid(RGB2GRB(Color.kWhite)).atBrightness(Percent.of(Constants.LEDConstants.BRIGHTPERCENT));
            }

        allianceColor.applyTo(m_underglow); 
        m_led.setData(m_ledBuffer);
        allianceComparable = alliance;
        }
        
        
    }

    public void setElevatorGradiant(){
        

        elevatorPattern = LEDPattern.gradient(GradientType.kContinuous,(RGB2GRB(Color.kRed)), RGB2GRB(Color.kWhite)).scrollAtRelativeSpeed(Percent.per(Second).of(25));
        //elevatorPattern = LEDPattern.gradient(GradientType.kContinuous,Color.kRed, Color.kWhite).synchronizedBlink(RobotController::getRSLState);
        elevatorPattern.applyTo(m_elevator); 
        //m_led.setData(m_ledBuffer);
    }

    public void setElevatorRSL(){

        elevatorPattern = LEDPattern.gradient(GradientType.kContinuous,(RGB2GRB(Color.kRed)), RGB2GRB(Color.kWhite)).synchronizedBlink(RobotController::getRSLState);
        elevatorPattern.applyTo(m_elevator); 
        //m_led.setData(m_ledBuffer);
    }

    public void setElevatorProgressLights(){
  

        LEDPattern basePattern = LEDPattern.solid(RGB2GRB(Color.kRed));
        LEDPattern progressPattern = LEDPattern.progressMaskLayer(()-> ((m_elevatorSubsystem.getEleEncoder() /Constants.ElevatorConstants.kElevatorMaxHeight)));
        //LEDPattern pCyan = LEDPattern.solid(RGB2GRB(Color.kCyan));
        //elevatorPattern = pCyan.mask(progressPattern).overlayOn(basePattern);
        elevatorPattern = progressPattern.overlayOn(basePattern);
        //elevatorPattern.applyTo(m_elevator);
        m_elevator = doubleColorProgress(m_elevator, (m_elevatorSubsystem.getEleEncoder() /Constants.ElevatorConstants.kElevatorMaxHeight), RGB2GRB(Color.kCyan), RGB2GRB(Color.kRed));
        }

        public void testProgressLights(){
  

            
            m_elevator = doubleColorProgress(m_elevator, LEDSetpoint, RGB2GRB(Color.kCyan), RGB2GRB(Color.kRed));
            //elevatorPattern.applyTo(m_elevator);
            }

    public AddressableLEDBufferView doubleColorProgress(AddressableLEDBufferView buffer, double progress, Color color1, Color color2){
        int buffLen = buffer.getLength();
        for(int i = 0; i<buffLen; i++){

            if(((double) i)/((double) buffLen) <= progress){
                buffer.setLED(i, color1);
            }else{
                buffer.setLED(i, color2);
            }
        }
        return buffer;
    }

    public void setElevatorSteps(){

        elevatorPattern = LEDPattern.steps(Map.of(0,RGB2GRB(Color.kRed),0.5, RGB2GRB(Color.kCyan))).scrollAtRelativeSpeed(Percent.per(Second).of(25));
        elevatorPattern.applyTo(m_elevator); 
        //m_led.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      LEDSetpoint = LEDSP.getDouble(LEDSetpoint);
      setUnderGlow(DriverStation.getAlliance().get());
      //setElevatorSteps();
      setElevatorSteps();
      //testProgressLights();
      if(DriverStation.isEnabled()){
        setElevatorProgressLights();
      }
      
      
      m_led.setData(m_ledBuffer);
    }

    
    private Color RGB2RBG(Color color) {
        //Our LEDs are GRB type, and as such will show green when we want our delightful red team color. 
        //In order to mitigate this, we will convert RGB to GBR for the colors we need to use.
        //Color resultColor = new Color(color.green,color.red,color.blue);
        Color resultColor = new Color(color.red,color.blue,color.green);
        
        return resultColor;
    }
    private Color RGB2GRB(Color color) {
        //Our LEDs are GRB type, and as such will show green when we want our delightful red team color. 
        //In order to mitigate this, we will convert RGB to GBR for the colors we need to use.
        //Color resultColor = new Color(color.green,color.red,color.blue);
        Color resultColor = new Color(color.green,color.red,color.blue);
        
        return resultColor;
    }

    
}
