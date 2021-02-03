#include <SPI.h>
#include <LoRa.h>

enum KissMarker {
  Fend = 0xc0,
  Fesc = 0xdb,
  Tfend = 0xdc,
  Tfesc = 0xdd
};

enum KissState {
  Void = 0,
  GetCmd,
  GetData,
  GetP,
  GetSlotTime,
  Escape
};

enum KissCmd {
  Data = 0x00,
  P = 0x02,
  SlotTime = 0x03,
  NoCmd = 0x80
};

#define DEFAULT_P             128
#define DEFAULT_SLOT_TIME     500

#define CFG_LORA_PIN_SS       5
#define CFG_LORA_PIN_RST      26
#define CFG_LORA_PIN_DIO0     14

#define CFG_LORA_FREQ         433.775E6
#define CFG_LORA_SYNC_WORD    0x34
#define CFG_LORA_BW           125e3
#define CFG_LORA_SF           12
#define CFG_LORA_CR           7
#define CFG_LORA_PWR          20
#define CFG_LORA_ENABLE_CRC   true

#define CONN_RETRY_SLEEP_MS   1000
#define LOOP_SLEEP_MS         10

KissState kissState_;
KissCmd kissCmd_;

// not used
byte csmaP_ = DEFAULT_P;
long csmaSlotTime_ = DEFAULT_SLOT_TIME;
long csmaSlotTimePrev_ = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  LoRa.setPins(CFG_LORA_PIN_SS, CFG_LORA_PIN_RST, CFG_LORA_PIN_DIO0);
  
  while (!LoRa.begin(CFG_LORA_FREQ)) {
    Serial.print(".");
    delay(CONN_RETRY_SLEEP_MS);
  }
  LoRa.setSyncWord(CFG_LORA_SYNC_WORD);
  LoRa.setSpreadingFactor(CFG_LORA_SF);
  LoRa.setSignalBandwidth(CFG_LORA_BW);
  LoRa.setCodingRate4(CFG_LORA_CR);
  LoRa.setTxPower(CFG_LORA_ENABLE_CRC);
  if (CFG_LORA_ENABLE_CRC) {
    LoRa.enableCrc();
  }
}

void loop() { 
  if (LoRa.parsePacket() > 0) {
    onRadioDataAvailable();
  }
  else {
    long currentTime = millis();
    if (currentTime > csmaSlotTimePrev_ + csmaSlotTime_ && random(0, 255) < csmaP_) {
      if (Serial.available()) {
        onSerialDataAvailable();
      }
    }
    csmaSlotTimePrev_ = currentTime;
  }
  delay(LOOP_SLEEP_MS);
}

void kissResetState()
{
  kissCmd_ = KissCmd::NoCmd;
  kissState_ = KissState::Void;
}

void onRadioDataAvailable() 
{
  Serial.write(KissMarker::Fend);
  Serial.write(KissCmd::Data);

  while (LoRa.available()) {
    byte rxByte = LoRa.read();

    if (rxByte == KissMarker::Fend) {
      Serial.write(KissMarker::Fesc);
      Serial.write(KissMarker::Tfend);
    }
    else if (rxByte == KissMarker::Fesc) {
      Serial.write(KissMarker::Fesc);
      Serial.write(KissMarker::Tfesc);
    }
    else {
      Serial.write(rxByte);
    }
  }
  Serial.write(KissMarker::Fend);
}

void onSerialDataAvailable() 
{ 
  while (Serial.available()) {
    
    int rxResult = Serial.read();
    if (rxResult == -1) break;
    
    byte rxByte = (byte)rxResult;

    switch (kissState_) {
      case KissState::Void:
        if (rxByte == KissMarker::Fend) {
          kissCmd_ = KissCmd::NoCmd;
          kissState_ = KissState::GetCmd;
        }
        break;
      case KissState::GetCmd:
        if (rxByte != KissMarker::Fend) {
          if (rxByte == KissCmd::Data) {
            LoRa.beginPacket();
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetData;
          }
          else if (rxByte == KissCmd::P) {
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetP;
          }
          else if (rxByte == KissCmd::SlotTime) {
            kissCmd_ = (KissCmd)rxByte;
            kissState_ = KissState::GetSlotTime;
          }
          else {
            kissResetState();
          }
        }
        break;
      case KissState::GetP:
        csmaP_ = rxByte;
        kissState_ = KissState::GetData;
        break;
      case KissState::GetSlotTime:
        csmaSlotTime_ = (long)rxByte * 10;
        kissState_ = KissState::GetData;
        break;
      case KissState::GetData:
        if (rxByte == KissMarker::Fesc) {
          kissState_ = KissState::Escape;
        }
        else if (rxByte == KissMarker::Fend) {
          if (kissCmd_ == KissCmd::Data) {
            LoRa.endPacket();
          }
          kissResetState();
        }
        else if (kissCmd_ == KissCmd::Data) {
          LoRa.write(rxByte);
        }
        break;
      case KissState::Escape:
        if (rxByte == KissMarker::Tfend) {
          LoRa.write(KissMarker::Fend);
          kissState_ = KissState::GetData;
        }
        else if (rxByte == KissMarker::Tfesc) {
          LoRa.write(KissMarker::Fesc);
          kissState_ = KissState::GetData;
        }
        else {
          kissResetState();
        }
        break;
      default:
        break;
    }
  }
}
