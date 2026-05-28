#define TRIG_PIN 12
#define ECHO_PIN 11
#define SAMPLE_INTERVAL_US 30
#define TEMPERATURE 32
#define HUMIDITY 79

int count=0;

float calcSpeedOfSound(){
  return 331.45f+0.606718f*TEMPERATURE+(0.0094208f+0.000077074f*TEMPERATURE)*HUMIDITY;
}

float SPEED_OF_SOUND=calcSpeedOfSound();

struct KalmanFilter{
  float q,r,x,p,k;

  void begin(float processNoise,float measureNoise,float initValue){
    q=processNoise;
    r=measureNoise;
    x=initValue;
    p=1.0f;
  }

  float filter(float value){
    p+=q;
    k=p/(p+r);
    x=x+k*(value-x);
    p=(1.0f-k)*p;
    return x;
  }
};

KalmanFilter kf;

void setup(){
  Serial.begin(9600);
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);
}

void loop(){
  float distance=readDistance();
  Serial.print(count);
  Serial.print(": Distance: ");
  Serial.println(distance);
  count++;
  delay(100);
}

float readDistance(){
  uint32_t duration=getEchoTime();
  return SPEED_OF_SOUND*duration*0.5e-4;
}

void test_1(uint8_t sampleCount){
  float data[sampleCount];
  float total=0;

  for(uint8_t i=0;i<sampleCount;i++){
    float distance=readDistance();
    data[i]=distance;
    total+=distance;
    delayMicroseconds(SAMPLE_INTERVAL_US);
  }

  float average=total/sampleCount;
  float variance=0;

  for(uint8_t i=0;i<sampleCount;i++){
    variance+=(data[i]-average)*(data[i]-average);
  }

  float deviation=sqrt(variance/sampleCount);

  Serial.print("Avg = ");
  Serial.print(average);
  Serial.print(" | Standard Deviation = ");
  Serial.println(deviation);
}

void test_2(uint8_t sampleCount,float referenceDistance){
  float data[sampleCount];
  float total=0;
  uint8_t validSamples=0;

  for(uint8_t i=0;i<sampleCount;i++){
    uint32_t duration=getEchoTime();

    if(duration==0){
      data[i]=-1;
    }else{
      float distance=SPEED_OF_SOUND*duration*0.5e-4;
      data[i]=distance;
      total+=distance;
      validSamples++;
    }

    delayMicroseconds(SAMPLE_INTERVAL_US);
  }

  if(validSamples==0){
    Serial.println("Avg = 0 | Standard Deviation = 0 | Error = 0 | Valid Rate = 0%");
    return;
  }

  float average=total/validSamples;
  float variance=0;

  for(uint8_t i=0;i<sampleCount;i++){
    if(data[i]!=-1){
      variance+=(data[i]-average)*(data[i]-average);
    }
  }

  float deviation=sqrt(variance/validSamples);
  float error=average-referenceDistance;
  float validRate=(float)validSamples*100.0f/sampleCount;

  Serial.print("Avg = ");
  Serial.print(average);
  Serial.print(" | Standard Deviation = ");
  Serial.print(deviation);
  Serial.print(" | Error = ");
  Serial.print(error);
  Serial.print(" | Valid Rate = ");
  Serial.print(validRate);
  Serial.println("%");
}

void test_3(){}

uint32_t getEchoTime(){
  digitalWrite(TRIG_PIN,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN,LOW);
  return pulseIn(ECHO_PIN,HIGH,30000);
}

uint32_t medianFilter(uint32_t arr[],uint8_t size){
  if(size==0)return 0;

  for(uint8_t i=1;i<size;i++){
    uint32_t temp=arr[i];
    int8_t j=i-1;

    while(j>=0&&arr[j]>temp){
      arr[j+1]=arr[j];
      j--;
    }

    arr[j+1]=temp;
  }

  return arr[size/2];
}
