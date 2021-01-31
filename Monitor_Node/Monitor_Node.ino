/*
 * Mointor code for the received tones and tokens
 */
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
#define SSID "Odat_Office"
#define PASSWORD "Odat_Office"
#define I2S_NUM           (0)         //i2s number
#define I2S_SAMPLE_RATE   (20480)     //i2s sample rate
#define I2S_SAMPLE_BITS   (32)        //i2s data bits
#define I2S_BUF_DEBUG     (1)         //enable display buffer for debug
#define I2S_READ_LEN      (4096)      //(16 * 1024) //I2S read buffer length
#define NEIGHBORS         (6)
#define MAX_SOUND_LEVEL   (5)
//Specific for a node
#define TONE_FREQ         (6600)     //Hz each node has its own frequency (4 KHz to 9.9 KHz)
String myID="ID9";
int frequency_step=(int)( double(TONE_FREQ*65536)/RTC_FAST_CLK_FREQ_APPROX);//freq = dig_clk_rtc_freq x SENS_SAR_SW_FSTEP / 65536
double neighborsFR[NEIGHBORS]={8301,5448,6250,7143,8500,5000};//,8301};
char neighborsID[NEIGHBORS][4]={"ID0","ID1","ID6","ID5","ID7","ID9"};//,"ID0"};
int myTopic[NEIGHBORS]={MAX_SOUND_LEVEL,MAX_SOUND_LEVEL,MAX_SOUND_LEVEL,MAX_SOUND_LEVEL,MAX_SOUND_LEVEL,MAX_SOUND_LEVEL};
//End of node specificity
const char* mqtt_server = "broker ip address";
WiFiClient masjid;
PubSubClient client(masjid);
double freqBinWidth=I2S_SAMPLE_RATE/I2S_READ_LEN;
int i2s_read_len = I2S_READ_LEN;
double* i2s_real;
double* i2s_imag;
//1 second-long of samples requires (sampling_rate*bits-per-sample/8) bytes of storage size
int* one_sec;//[I2S_SAMPLE_RATE * 4];
int sound_levels[NEIGHBORS]={MAX_SOUND_LEVEL,MAX_SOUND_LEVEL,MAX_SOUND_LEVEL,MAX_SOUND_LEVEL,MAX_SOUND_LEVEL,MAX_SOUND_LEVEL};
int token_level;
arduinoFFT FFT;// = arduinoFFT(i2s_real, i2s_imag, i2s_read_len, I2S_SAMPLE_RATE);
void i2s_init(void) {
  int i2s_num = I2S_NUM;
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX ),//| I2S_MODE_TX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  I2S_SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)I2S_SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,//I2S_CHANNEL_FMT_ALL_RIGHT,//I2S_CHANNEL_FMT_ONLY_LEFT,//I2S_FORMAT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S,//I2S_COMM_FORMAT_I2S_LSB,I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,//8 is enough the maximum
    .dma_buf_len = 1024,//should be at most 1024
    .use_apll = 1,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
    .bck_io_num = 32,   // Serial Clock (SCK)(BCLK)
    .ws_io_num = 25,    // Word Select (WS)(LRCL)
    .data_out_num = I2S_PIN_NO_CHANGE, // not used (only for speakers)
    .data_in_num = 33   // Serial Data (SD)(DOUT)
  };

  i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL); //install and start i2s driver
  i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
}
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
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
  char *str;
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
  //2. listener 1883 192.168.0.105 ----port and local_machine IP
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(myID.c_str())) {
      Serial.println("connected");
      // Subscribe
      char *tmp;
      for(int i=0;i<NEIGHBORS;i++){//subscribe to the topics of all neighbers 
        tmp=neighborsID[i];
        client.subscribe(tmp,0);
      }      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second");
      // Wait 0.5 seconds before retrying
      delay(500);
    }
  }
}

void fire_sound(int level){//(20 ms)-width tone
  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 2, SENS_DAC_INV2_S);
  dac_output_enable(DAC_CHANNEL_2);
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL, 0);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, frequency_step, SENS_SW_FSTEP_S);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE2, 0/*scale*/, SENS_DAC_SCALE2_S);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC2, 0/*offset*/, SENS_DAC_DC2_S);
  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 2, SENS_DAC_INV2_S);
  delay(20);//20 ms
  CLEAR_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);//CW silence
}

void setup() {  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(500);
    ESP.restart();
  }
  client.setServer(mqtt_server, 1883);  
  client.setCallback(callback);
  pinMode(2, OUTPUT);
  i2s_real = (double*) malloc(I2S_READ_LEN * sizeof(double)); //8 bytes (double)
  i2s_imag = (double*) malloc(I2S_READ_LEN * sizeof(double));
  one_sec = (int*) malloc(I2S_SAMPLE_RATE * sizeof(int));//assuming sample size is 4 bytes
  FFT = arduinoFFT(i2s_real, i2s_imag, i2s_read_len, I2S_SAMPLE_RATE);
  i2s_init();
  Serial.println("Started up");
}

void loop() {
  if (!client.connected()) {//conecet to broker
    reconnect();
  }
  client.loop();
  for(int i=0;i<5;i++){
    if (!client.connected()) {//conecet to broker
      reconnect();
    }
    client.loop();
    delay(2000);
  }     
  for (;;) {
    //check for next trigger
  }
}
