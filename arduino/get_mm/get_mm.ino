#define FRW_LFT_IK 9
#define FRW_RGT_IK 8
#define BKW_LFT_IK 7
#define LFT_FRW_IK 6
#define LFT_BKW_IK 5
#define BKW_RGT_IK 4
#define RGT_BKW_IK 3
#define RGT_FRW_IK 2

String get_output()
{
  String output;
  output = String(analogRead(A1)) + '\n' 
         + String(analogRead(A3)) + '\n' 
         + String(analogRead(A5)) + '\n' 
         + String(analogRead(A7)) + '\n' 
         + String(digitalRead(FRW_LFT_IK)) + '\n' 
         + String(digitalRead(FRW_RGT_IK)) + '\n' 
         + String(digitalRead(BKW_LFT_IK)) + '\n' 
         + String(digitalRead(LFT_FRW_IK)) + '\n' 
         + String(digitalRead(LFT_BKW_IK)) + '\n' 
         + String(digitalRead(BKW_RGT_IK)) + '\n' 
         + String(digitalRead(RGT_BKW_IK)) + '\n' 
         + String(digitalRead(RGT_FRW_IK)) + '\n'
         + "<";
  return output;
}

void setup()
{
  pinMode(FRW_LFT_IK, INPUT_PULLUP);
  pinMode(FRW_RGT_IK, INPUT_PULLUP);
  pinMode(BKW_LFT_IK, INPUT_PULLUP);
  pinMode(LFT_FRW_IK, INPUT_PULLUP);
  pinMode(LFT_BKW_IK, INPUT_PULLUP);
  pinMode(BKW_RGT_IK, INPUT_PULLUP);
  pinMode(RGT_BKW_IK, INPUT_PULLUP);
  pinMode(RGT_FRW_IK, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("Sensors module for cleaning robot. V. 0.9");
  Serial.println("");
  Serial.println(get_output());
  Serial.println("");
}

void loop()
{
  if (Serial.available() > 0) {
    if (Serial.read() == '>') {
      Serial.println(get_output());
    }
  }
}
