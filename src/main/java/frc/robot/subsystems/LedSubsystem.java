package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED ledBar;
    private final AddressableLEDBuffer ledBuffer;
    private final ElevatorSubsystem elevator;
    private final ScorerSubsystem scorer; // Added dependency

    private LEDPattern allianceLED;
    private LEDPattern currentPattern;
    private enum LedState { BLANK, RAINBOW, SCROLL, BREATHING, ALLIANCE_SOLID, GREEN, YELLOW }
    private LedState currentState = LedState.BLANK;

    public LedSubsystem(ElevatorSubsystem elevator, ScorerSubsystem scorer) {
        this.elevator = elevator;
        this.scorer = scorer;

        // Initialize alliance color
        setAllianceColor();

        // Initialize LED hardware
        ledBar = new AddressableLED(LEDConstants.kLEDBarPWM);
        ledBar.setLength(LEDConstants.ledLength);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);

        // Start with blank pattern
        setBlank();
        ledBar.setData(ledBuffer);
        ledBar.start();
    }

    // @Override
    // public void periodic() {
    //     // Update LED state based on robot mode and conditions
    //     if (DriverStation.isDisabled()) {
    //         setScroll();
    //     } else if (DriverStation.isAutonomous()) {
    //         setRainbow();
    //     } else if (DriverStation.isTeleop()) {
    //         double timeRemaining = DriverStation.getMatchTime();
    //         if (timeRemaining > 0 && timeRemaining < 20) {
    //             setBreathing();
    //         } else if (scorer.hasCoral()) {
    //             setGreen();
    //         } else if (scorer.ongoingCoral()) {
    //             setYellow();
    //         } else {
    //             setAllianceSolid();
    //         }
    //     }

    //     // Apply the current pattern if it changed
    //     if (currentPattern != null) {
    //         currentPattern.applyTo(ledBuffer);
    //         ledBar.setData(ledBuffer);
    //     }
    // }

    private void setAllianceColor() {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        allianceLED = (alliance == DriverStation.Alliance.Red)
            ? LEDPattern.solid(Color.kRed)
            : LEDPattern.solid(Color.kBlue);
    }

    // Public methods to set specific patterns
    public void setBlank() {
        if (currentState != LedState.BLANK) {
            currentPattern = LEDPattern.solid(Color.kBlack);
            currentState = LedState.BLANK;
        }
    }

    public void setRainbow() {
        if (currentState != LedState.RAINBOW) {
            currentPattern = LEDPattern.rainbow(255, 128).atBrightness(Percent.of(100));
            currentState = LedState.RAINBOW;
        }
    }

    public void setScroll() {
        if (currentState != LedState.SCROLL) {
            currentPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kCoral, Color.kWheat)
                .scrollAtRelativeSpeed(Percent.per(Second).of(25));
            currentState = LedState.SCROLL;
        }
    }

    public void setBreathing() {
        if (currentState != LedState.BREATHING) {
            currentPattern = allianceLED.breathe(Second.of(2.5));
            currentState = LedState.BREATHING;
        }
    }

    public void setAllianceSolid() {
        if (currentState != LedState.ALLIANCE_SOLID) {
            currentPattern = allianceLED;
            currentState = LedState.ALLIANCE_SOLID;
        }
    }

    public void setGreen() {
        if (currentState != LedState.GREEN) {
            currentPattern = LEDPattern.solid(Color.kGreen);
            currentState = LedState.GREEN;
        }
    }

    public void setYellow() {
        if (currentState != LedState.YELLOW) {
            currentPattern = LEDPattern.solid(Color.kYellow);
            currentState = LedState.YELLOW;
        }
    }

    public void setElevatorProgress() {
        currentPattern = LEDPattern.progressMaskLayer(() -> elevator.getPosition() / ElevatorConstants.kElevatorMaxHeightRaw)
            .overlayOn(LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kTurquoise, Color.kNavy));
        currentState = null; // Custom state, not tracked
    }

    public void stop() {
        ledBar.stop();
    }

    public void start() {
        ledBar.start();
    }
}

// public class LedSubsystem extends SubsystemBase {

//     private static ElevatorSubsystem elevator = new ElevatorSubsystem();
            
//             private static AddressableLED ledBar; // instance of the led bar class
        
//             // Buffer or the holder of the color's data and its variable
//             private static AddressableLEDBuffer led_red_alliance;
//             private static AddressableLEDBuffer led_blue_alliance;
//             private static AddressableLEDBuffer led_red_blue;
//             private static AddressableLEDBuffer led_blank;
//             private static AddressableLEDBuffer led_green;
//             private static AddressableLEDBuffer led_yellow;
//             // private static AddressableLEDBuffer led_dynamic_msg;
//             private static AddressableLEDBuffer rainbow_buffer;
//             private static AddressableLEDBuffer elevator_progress_buffer;
//             private static AddressableLEDBuffer scroll_buffer;
//             private static AddressableLEDBuffer breath_buffer;
//             private static AddressableLEDBuffer alliance_buffer;


//             private static LEDPattern scrollBase = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kCoral, Color.kWheat);
//             private static LEDPattern allianceLED;
        
        
//             // private static int dynamicCount = 0; // counter for dynamic messages
//             // private static int rainbowFirstPixelHue = 0; // tracks the hue for the first pixel in the rainbow effect
//             // public static boolean dynamicEffect = false; // a tracking varible for checking dynamic effect
        
//             // a block that runs once when class is initially loaded
//             static {
//                 setAllianceColor();

//                 ledBar = new AddressableLED(LEDConstants.kLEDBarPWM); // initializes the led bar object with the given power port
//                 ledBar.setLength(LEDConstants.ledLength);
        
//                 // set buffers to their specified length
        
//                 led_red_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_blue_alliance = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_red_blue = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_blank = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 rainbow_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 elevator_progress_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 scroll_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 breath_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 alliance_buffer = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_green = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 led_yellow = new AddressableLEDBuffer(LEDConstants.ledBufferLength);
//                 // led_dynamic_msg = new AddressableLEDBuffer(LEDConstants.ledBufferLength); // specific blinking message
        
//                 // initial message for led buffers
//                 for (int i = 0 ; i < LEDConstants.ledLength; i++){
//                     // pre-set all message buffers during initilization
//                     led_red_alliance.setLED(i, Color.kRed);
//                     led_blue_alliance.setLED(i, Color.kBlue);
//                     led_green.setLED(i, Color.kGreen);
//                     led_yellow.setLED(i, Color.kYellow);
        
//                     // using bitwise-AND operators .... (alternating red-blue led)
//                     if (((i & 3) == 0) // if the last two binary value of i & 3 is the same as 0's
//                      || ((i & 3) == 2) // if if the last two binary value of i & 3 is the same as 2's
//                      ){
//                         led_red_blue.setLED(i, Color.kRed);
//                     }  else {
//                         led_red_blue.setLED(i, Color.kBlue);
//                     }
        
//                     led_blank.setLED(i, Color.kBlack);
//                     // led_dynamic_msg.setLED(i, Color.kBlack); // no pre-set for dynamic msg
//                 }
        
//                 ledBar.setData(led_blank); // displays the black-led at first
//                 ledBar.start(); // activates the led strip
//             }
        
//             public static void stopLedBar(){
//                 ledBar.stop(); // turns off
//             }
        
//             public static void startLedBar(){
//                 ledBar.start(); // turns on
//             }
        
//             public static void setBlankMsg(){
//                 // dynamicEffect = false;
//                 ledBar.setData(led_blank); // doesn't light up
//             }
        
//             public static void setRedBlueMsg(){
//                 // dynamicEffect = false; 
//                 ledBar.setData(led_red_blue); // lights up red_blue
//             }
        
//             public static void setGreenMsg(){
//                 // dynamicEffect = false;
//                 ledBar.setData(led_green); // lights up green
//             }

//             public static void setYellowMsg(){
//                 // dynamicEffect = false;
//                 ledBar.setData(led_yellow); // lights up green
//             }
        
//             public static void setAllianceColor(){ // sets allaince color in led
//                 DriverStation.Alliance friendlyAlliance = DriverStation.getAlliance().get();
//                 if(friendlyAlliance == DriverStation.Alliance.Red){
//                     allianceLED = LEDPattern.solid(Color.kRed);
//                 }
//                 else if(friendlyAlliance == DriverStation.Alliance.Blue){
//                     allianceLED = LEDPattern.solid(Color.kBlue);
//                 }
//                 else {
//                     allianceLED = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
//                 }
//             }

//             public static void setAllianceSolid(){
//                 allianceLED.applyTo(alliance_buffer);
//                 ledBar.setData(alliance_buffer);
//                 ledBar.setData(alliance_buffer);
//             }
        
//             public static void scrollMsg(){
//                 LEDPattern pattern = scrollBase.scrollAtRelativeSpeed(Percent.per(Second).of(25));
        
//                 pattern.applyTo(scroll_buffer);
        
//                 ledBar.setData(scroll_buffer);
//             }
        
//             public static void setBreathingMsg(){
//                 LEDPattern breathing = allianceLED.breathe(Second.of(2.5));
        
//                 breathing.applyTo(breath_buffer);
        
//                 ledBar.setData(breath_buffer);
//             }
        
//             public static void setRainbow(){
//                 LEDPattern rainbow = LEDPattern.rainbow(255, 128).atBrightness(Percent.of(100));
//                 rainbow.applyTo(rainbow_buffer);
//                 ledBar.setData(rainbow_buffer);
//             }
        
//             public static void elevatorMsg(){
//                 LEDPattern elevatorGradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kTurquoise, Color.kNavy);
//                 LEDPattern elevatoPattern = LEDPattern.progressMaskLayer(() -> elevator.getPosition()/ElevatorConstants.kElevatorMaxHeightRaw).overlayOn(elevatorGradient);
//                 elevatoPattern.applyTo(elevator_progress_buffer);
//                 ledBar.setData(elevator_progress_buffer);
//             }

//     // public static void setRainbow(){ // sets rainbow led
//     //     // For every pixel
//     //     for (var i = 0; i < led_dynamic_msg.getLength(); i++) {
//     //       // Calculate the hue - hue is easier for rainbows because the color
//     //       // shape is a circle so only one value needs to precess
//     //       final var hue = (rainbowFirstPixelHue + (i * 180 / led_dynamic_msg.getLength())) % 180;
//     //       // Set the value
//     //       led_dynamic_msg.setHSV(i, hue, 255, 128);
//     //     }
//     //     // Increase by to make the rainbow "move"
//     //     rainbowFirstPixelHue += 3;
//     //     // Check bounds
//     //     rainbowFirstPixelHue %= 180;
//     // }

//     // public static void setDynamicMsg(){
//     //     if ( dynamicCount < 1000 ){
//     //         dynamicCount++;
//     //     }
//     //     setRainbow();
//     //     ledBar.setData(led_dynamic_msg);
//     // }
// }
