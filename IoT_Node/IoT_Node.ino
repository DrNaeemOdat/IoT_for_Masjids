#include <arduinoFFT.h>
#include <stdio.h>
#include <string.h>
#include "driver/i2s.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include <driver/dac.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define SSID "your access point SSID"
#define PASSWORD "password of your access point"
#define I2S_NUM           (0)         //i2s number
#define I2S_SAMPLE_RATE   (20480)     //i2s sample rate
#define I2S_SAMPLE_BITS   (32)        //i2s data bits
#define I2S_BUF_DEBUG     (1)         //enable display buffer for debug
#define I2S_READ_LEN      (4096)      //I2S read buffer length
#define NEIGHBORS         (2)
#define MAX_SOUND_LEVEL   (5)
//Specific for a node
////ID0
//#define TONE_FREQ         (8305)    //Hz each node has its own frequency (4 KHz to 9.9 KHz)
//String myID="ID0";
//float addedThreshold=2;
//ID1
#define TONE_FREQ       (5450)        //Hz each node has its own frequency (4 KHz to 9.9 KHz)
String myID="ID1";
float addedThreshold=2;
double neighborsFR[NEIGHBORS]={8305,5450};
char neighborsID[NEIGHBORS][4]={"ID0","ID1"};
double neighborsAvg[NEIGHBORS]={0,0};
int frequency_step=(int)( double(TONE_FREQ*65536)/RTC_FAST_CLK_FREQ_APPROX);//freq = dig_clk_rtc_freq x SENS_SAR_SW_FSTEP / 65536
int myTopic[NEIGHBORS];//={MAX_SOUND_LEVEL,MAX_SOUND_LEVEL};
const char* mqtt_server = "IP address";
WiFiClient masjid;
PubSubClient client(masjid);
double freqBinWidth=I2S_SAMPLE_RATE/I2S_READ_LEN;
int i2s_read_len = I2S_READ_LEN;
double* i2s_real;
double* i2s_imag;
//1 second-long of samples requires (sampling_rate*bits-per-sample/8) bytes of storage size
int* one_sec;
int sound_levels[NEIGHBORS]={MAX_SOUND_LEVEL,MAX_SOUND_LEVEL};
int token_level;
arduinoFFT FFT;
void i2s_init(void) {
  int i2s_num = I2S_NUM;
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX ),
    .sample_rate =  I2S_SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)I2S_SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,//8 is enough the maximum
    .dma_buf_len = 1024,//should be at most 1024
    .use_apll = false,//1,
    .tx_desc_auto_clear = false,//true,
    .fixed_mclk = 0
  };
  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
    .bck_io_num = 32,   // Serial Clock (SCK)(BCLK)
    .ws_io_num = 25,    // Word Select (WS)(LRCL)
    .data_out_num = I2S_PIN_NO_CHANGE, // not used 
    .data_in_num = 33   // Serial Data (SD)(DOUT)
  };

  i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL); //install and start i2s driver
  i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
}
void callback(char* topic, byte* message, unsigned int length) {
  String subscribedTo=String(topic);
  int indx=-1;
  for(int i=0;i<NEIGHBORS;i++){//What topic we have received
    if(String(neighborsID[i])==subscribedTo){
      indx=i;
      break;
    }
  }
  int found=0;
  char *p = (char*)message;
  char *str=NULL;
  while ((str = strtok_r(p, " ", &p)) != NULL){ // delimiter is the white space
    if(String(str)==myID){
      found=1;
      break;
    }
  }
  if(indx>=0 && found){
    sound_levels[indx]=token_level;//save the level in the array of levels
  }
}

void reconnect() {
  //In the mosquitto broker configuration file (mosquitto.conf)
  //add :
  //1. allow_anonymous true
  //2. listener 1883 mqtt server IP ----port and local_machine IP
  // Loop until we're reconnected
  while (!client.connected()) {
    // Attempt to connect
    if (client.connect(myID.c_str())) {
      // Subscribe
      char *tmp;
      for(int i=0;i<NEIGHBORS;i++){//subscribe to the topics of all neighbers 
        tmp=neighborsID[i];
        client.subscribe(tmp,0);
      }      
    } else {
      // Wait 0.5 seconds before retrying
      delay(500);
    }
  }
}

void fire_sound(int level){//40ms-width tone
  noInterrupts();
  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 2, SENS_DAC_INV2_S);
  dac_output_enable(DAC_CHANNEL_2);
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL, 0);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, frequency_step, SENS_SW_FSTEP_S);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE2, 0/*scale*/, SENS_DAC_SCALE2_S);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC2, 0/*offset*/, SENS_DAC_DC2_S);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 2, SENS_DAC_INV2_S);
  delay(140);//40 ms
  CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);//CW silence
  interrupts();
}

void setup() {  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(500);
    ESP.restart();
  }
  client.setServer(mqtt_server, 1883);  
  client.setCallback(callback);
  pinMode(2, OUTPUT);
  pinMode(34, INPUT);//PushButton input
  i2s_real = (double*) malloc(I2S_READ_LEN * sizeof(double)); //8 bytes (double)
  i2s_imag = (double*) malloc(I2S_READ_LEN * sizeof(double));
  one_sec = (int*) malloc(I2S_SAMPLE_RATE * sizeof(int));//assuming sample size is 4 bytes
  FFT = arduinoFFT(i2s_real, i2s_imag, i2s_read_len, I2S_SAMPLE_RATE);
  i2s_init();
}

void loop() {
  /*
     this routine runs when the node is triggred by Azan timming board controller
     when the node finished its tone sending, it listens to other nodes tones
     For testing purpose, a pushbutton is used for synchronizing the procedure
  */    
  while(1){
    if(digitalRead(34)==LOW){
      delay(30);      
      if(digitalRead(34)==LOW){
        break;//Go ahead with sound level setting
      }
    } 
  }
  //End of trigger
  for(int i=0;i<NEIGHBORS;i++){//Initialize the tone level for each neighbor
    myTopic[i]=MAX_SOUND_LEVEL;     
  }  
  int *ptr = one_sec;
  size_t bytes_read;
  //Calculate background noise obn each neighbors frequency
  i2s_read((i2s_port_t)I2S_NUM, (void*) ptr, 8192/*dma_buffer_length*number_of_buffers*/, &bytes_read, portMAX_DELAY);
  for (int j = 0; j < i2s_read_len; j++) {
    i2s_real[j] = ptr[j]>>11; //convert to double and correct it
  }
  memset(i2s_imag, 0.0, i2s_read_len * 8); //To avoid overflow
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  for(int i=0;i<NEIGHBORS;i++){
    int indx=ceil(neighborsFR[i]/freqBinWidth);
    neighborsAvg[i] = i2s_real[indx];     
  }
  //End of noise calculation
  memset(one_sec, 0, I2S_SAMPLE_RATE * sizeof(int));
  digitalWrite(2, HIGH);
  fire_sound(5);//140 ms duration 
  digitalWrite(2, LOW);
  //delay(200);//200 ms, to make sure the tone has arrived from other nodes
  for (int i = 0; i < 10; i++) {
    ptr = &one_sec[2048 * i];
    i2s_read((i2s_port_t)I2S_NUM, (void*) ptr, 8192/*dma_buffer_length*number_of_buffers*/, &bytes_read, portMAX_DELAY);
  }
  ptr = one_sec;
  int windows=(int)(I2S_SAMPLE_RATE/i2s_read_len);
  for (int i = 0; i < windows; i++) {//loop over all the 1 second samples in windows of i2s_read_len size      
    for (int j = 0; j < i2s_read_len; j++) {
      i2s_real[j] = ptr[j]>>11;/// 0xfff; //convert to double and corrections
    }
    memset(i2s_imag, 0.0, i2s_read_len * 8); //To avoid overflow
    FFT.DCRemoval();
    FFT.Windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);//FFT_WIN_TYP_WELCH, FFT_FORWARD);
    FFT.Compute(FFT_FORWARD);
    FFT.ComplexToMagnitude();
    for(int i=0;i<NEIGHBORS;i++){
      int indx=ceil(neighborsFR[i]/freqBinWidth);
      if(i2s_real[indx] > neighborsAvg[i]+10000*addedThreshold){
        myTopic[i]=MAX_SOUND_LEVEL;
      }        
    }
    ptr += i2s_read_len; //work on the next window (4096 samples)
  }
  for (int level = MAX_SOUND_LEVEL; level >0; level--) {//5 levels of sound
    digitalWrite(2, HIGH);
    fire_sound(level);//140 ms duration 
    digitalWrite(2, LOW);
    //delay(200);//200 ms, to make sure the tone has arrived from other nodes
    for (int i = 0; i < 10; i++) {
      ptr = &one_sec[2048 * i];
      i2s_read((i2s_port_t)I2S_NUM, (void*) ptr, 8192/*dma_buffer_length*number_of_buffers*/, &bytes_read, portMAX_DELAY);
    }
    ptr = one_sec;
    int windows=(int)(I2S_SAMPLE_RATE/i2s_read_len);
    for (int i = 0; i < windows; i++) {//loop over all the 1 second samples in windows of i2s_read_len size      
      for (int j = 0; j < i2s_read_len; j++) {
        i2s_real[j] = ptr[j]>>11;/// 0xfff; //convert to double and corrections
      }
      memset(i2s_imag, 0.0, i2s_read_len * 8); //To avoid overflow
      FFT.DCRemoval();
      FFT.Windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);//FFT_WIN_TYP_WELCH, FFT_FORWARD);
      FFT.Compute(FFT_FORWARD);
      FFT.ComplexToMagnitude();
      for(int i=0;i<NEIGHBORS;i++){
        int indx=ceil(neighborsFR[i]/freqBinWidth);
        if(i==0)
          Serial.println(i2s_real[indx]);        
        if(i2s_real[indx] > neighborsAvg[i]+10000*addedThreshold){
          myTopic[i]=level;
        }        
      }
      ptr += i2s_read_len; //work on the next window (4096 samples)      
    }
  }

  String token=" ";
  for(int i=0;i<NEIGHBORS;i++){
    token+=String(myTopic[i]);
    token+=" ";
  }

  if (!client.connected()) {//conecet to broker
    reconnect();
  }
  client.loop();
  client.publish(myID.c_str(),(const uint8_t*)token.c_str(),token.length(),true);  
  for(int i=0;i<5;i++){//during this second node gets info about its sound levels from the broker
    if (!client.connected()) {//conecet to broker
      reconnect();
    }
    client.loop();
    delay(100);
  }
  //calculate_level();    
}
