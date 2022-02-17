//define stepper variables
const int stepPin = 4;
const int dirPin = 2;
int Delay = 5; // for the stepper motor
bool stepperHome = false;
const int hallEffectSensor = 15;

//define pressure transducer variables
float pressureInput = 34; 
const int pressureZero = 385;
const int pressureMax = 3465;
const int pressuretransducerMaxPSI = 60;
float pressureValue = 0;
double pressureArray[20]= {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
int pressureArrayIndex = 0;
double averagePressure;
float pressure_stdDev;
float pressureSum;

//define p controller variables
double kp;          //set value of proportional coefficient
int pressureSetpoint; //set point
float PID_error = 0;       
int steps = 0;
int loopDelay = 500; // delay between P reassesment

//define flow sensor variables
volatile int flow_frequency; // Measures flow sensor pulses
double l_min; // Calculated litres/minute
double flowArray [20] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float calibrationFactor = 5.5;
int flowArrayIndex = 0;
int flowReadDelay = 10000;
unsigned char flowsensor = 33; // Sensor Input
unsigned long oldTime;
double averageLPM; //the averaged flow rate value
float flow_stdDev = 0;
float flowSum;
int counter = 0;
float newLPM;
//define system state variables
int system_state; //1 = searching_state,2 = listening_state, 3 = tuning_state, 4 = display air flow
//state1 : runs once then goes to state 2,state 2 to state 3 : awaits serial data input,state 3 to state 4 : after stability is achieved

void moveStepper (int TimeDelay) { //move the stepper motor
  digitalWrite(stepPin, HIGH);
  delay(TimeDelay);
  digitalWrite(stepPin, LOW);
  delay(TimeDelay);
  } 

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT); //set pinmodes
  pinMode(dirPin, OUTPUT);
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
  attachInterrupt(15, ISR_go_home, FALLING);//Initialize the intterrupt pin
  attachInterrupt(33,ISR_flow, RISING); // Setup Interrupt
  sei(); // Enable interrupts
  flow_frequency = 0;
  oldTime = 0;

  while(stepperHome == false)//move stepper from current position to home
    moveStepper(1);
  for(int i = 0; i < 640; i++ )
    moveStepper(1);  
  Serial.println("Home"); 
  system_state = 2;   
}

void loop() {  
  float pressureResult = get_pressureValue();   
  PID_error = abs(pressureSetpoint-pressureValue); //calculate pid error
  steps = round((PID_error * kp));  //calculate steps required by stepper motor to achieve set pressure
  get_pressure_average();
  pressure_stdDev = get_pressure_stdDev();
  
  display_values();

  if (system_state == 2)
   listening_state();
  else if (system_state == 3)
   tuning_state ();
  else if (system_state == 4)
   air_flow_reading();
//  Serial.print("\t");
//  Serial.print("state = ");
//  Serial.println(system_state);
  delay(loopDelay);
}

double pressureArrayAverage(){//calculate the average pressure
  double p =0;
  int pressureCounter = 0;
  for (int x = 0 ; x < 20 ; x++)
    if (pressureArray[x] != 0){
      p = p + pressureArray[x];
      pressureCounter++;
    }
  if (pressureCounter > 0){
     return p/pressureCounter;
     }
  else {
    return 0;
    }
  }

 void display_values(){
    Serial.print(pressureValue);      //display pressure in psi
    Serial.print("\t");
    //Serial.print(steps);              //display number of steps
    //Serial.print("\t");
    Serial.print(averagePressure);//display average pressure
    Serial.print("\t");
    Serial.print(pressure_stdDev);//display standard deviation
    Serial.print("\t");
    Serial.print(pressureSetpoint); //display the pressure setpoint
    Serial.print("\t ");
    Serial.print(l_min);
    Serial.print("\t ");
    Serial.print(averageLPM); // Print litres/minute
    Serial.print("\t ");
    Serial.print(flow_stdDev); 
    Serial.print("\t ");
    Serial.println(newLPM);  
  }
float get_pressure_stdDev(){//calculate standard deviation
    float pressure_stdDev = 0;
    float pressureSum = 0;
    for(int a = 0; a < 20; a++){
      pressureSum += pow((pressureArray[a] - averagePressure),2);
     }
    pressure_stdDev = (sqrt(pressureSum/20)); 
    return pressure_stdDev;
}
float get_pressureValue(){//calculate pressure in psi
    pressureValue = analogRead(pressureInput); //read pressure
    pressureValue = ((pressureValue - pressureZero)*pressuretransducerMaxPSI)/(pressureMax - pressureZero); //convert read pressure to psi
    if (pressureValue < 0) //set all negative pressures to zero
      pressureValue = 0;
    return pressureValue;
  }

void listening_state(){   //get serial data input and assign kp value
  while(Serial.available() == 0)
    pressureSetpoint = Serial.parseInt();
    kp_values();
  if (pressureSetpoint > 0)
    system_state = 3;
}
void kp_values(){//p controller coefficient values for defined pressureSetpoint
  if (pressureSetpoint == 5) 
      kp=0.32;
  else if (pressureSetpoint == 10)
      kp=0.25;
  else if (pressureSetpoint == 15)
      kp=0.25;
  else if (pressureSetpoint == 20)
      kp=0.25;
  else if (pressureSetpoint == 25) 
      kp=0.23;
  else if (pressureSetpoint == 30) 
      kp=0.25;
  else if (pressureSetpoint == 35) 
      kp=0.17;
  else if (pressureSetpoint == 40) 
      kp=0.17;        
}
void tuning_state(){//move the stepper motor CW/CCW to achieve the pressure setpoint
  if ((pressureSetpoint-pressureValue) > 0){
    digitalWrite(dirPin,LOW); //move the stepper clockwise
    for(int x = 0; x < steps;x++ )
      moveStepper(5);
      }
  else{
   digitalWrite(dirPin,HIGH); //move the motor anticlockwise
   for(int y = 0; y < steps;y++ )
      moveStepper(5);
      }
  if((pressure_stdDev < 0.55)&&(PID_error <= 2)) {
        system_state = 4;
        counter = 0;
        flow_frequency = 0;
        oldTime = millis();
        flow_stdDev = 10;
        for (int i=0;i<20;i++)
          flowArray[i]=0.0;
  }  
}

void air_flow_reading(){//calculate the flow rate in litres /minute
  // Every second, calculate and print litres/minute
      if((millis() - oldTime) >= 1000){
        l_min = (flow_frequency / calibrationFactor); // (Pulse frequency) / 5.5Q = flowrate in L/minute
        oldTime = millis(); // Updates oldTime
        get_flow_average();
        flow_stdDev = get_flow_stdDev();
        flow_frequency = 0; // Reset Counter 
      }
   if(flow_stdDev < 4){
    newLPM = ((averageLPM*7.2638)-112.51);
    newLPM = 0;
    l_min = 0;
    averageLPM = 0;
    flow_stdDev =0;
    flush_serial();
    for (int k=0;k<20;k++)
      pressureArray[k]=0.0;
    system_state = 2;}
}

void ISR_go_home(){//interrupt for hall effect sensor to detect the magnet
   stepperHome = true;
}
void ISR_flow () // Flow sensor Interrupt function
{
   flow_frequency++;  
}
void get_pressure_average(){
  pressureArray[pressureArrayIndex] = pressureValue;
  averagePressure = pressureArrayAverage();
  pressureArrayIndex = ((pressureArrayIndex + 1)% 20);
}
void flush_serial(){
  while(Serial.available())
    Serial.parseInt();
}
double flowArrayAverage (){
  double F = 0;
  int flowValues = 0;
  for (int i = 0; i < 20; i++){
    if (flowArray[i] != 0){
      F = F + flowArray[i];
      flowValues ++;
    }
  }
  if (flowValues > 0){
    return F/flowValues;
  }
  else {
    return 0;
  }
}
void get_flow_average(){
  flowArray [flowArrayIndex] = l_min;
  averageLPM = flowArrayAverage ();
  flowArrayIndex = (flowArrayIndex + 1) % 20;
}

float get_flow_stdDev(){//calculate standard deviation
    float flow_stdDev = 0;
    float flowSum = 0;
    for(int b = 0; b < 20; b++){
      flowSum += pow((flowArray [b] - averageLPM),2);
     }
    return sqrt(flowSum/20);
}
