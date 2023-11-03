
const int b_Pin = 2; // The pin pushbutton is connected to
int b_State = 0;     // Button state initialised in this variable
bool prog_Running = true; // Script running controll initialised in this variable

void setup() 
{
  pinMode(b_Pin, INPUT);
  Serial.begin(9600);
}

void loop() 
{
  
  b_State = digitalRead(b_Pin);

  if (b_State == LOW) 
  {
    
    if (prog_Running) 
    {
      
      Serial.println("STOP"); // Send a signal to MATLAB to stop the script
      prog_Running = false;
      
    }

  } 
  
  else 
  {
    
    if (!prog_Running) 
    {
      
      Serial.println("RESUME"); // Send a signal to MATLAB to resume the script
      prog_Running = true;

    }

  }

  delay(100); // Debounce the button (adjust as needed)

}
