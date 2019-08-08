#include <SPI.h>
#include <RH_NRF24.h>

RH_NRF24 nrf24;

void setup() 
{
  analogWrite(5,150);
  Serial.begin(9600);
  while (!Serial);                             // Wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");             // Defaults after init are 2.402 GHz(channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  else 
    Serial.println("noice");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");    
  else
    Serial.println("big noice");
}

void loop()
{
  Serial.println("Sending to nrf24_server");   // Send a message to nrf24_server
  uint8_t data[] = "Hello World!";
  nrf24.send(data, sizeof(data));
  nrf24.waitPacketSent();
  delay(1000);
}
