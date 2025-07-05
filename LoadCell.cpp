// LoadCell.cpp
#include "LoadCell.h"

#include <SPI.h>
#include <ADS1256.h>
#include "Main.h"

static const float ADC_CLOCK_MHZ = 7.68f; // crystal frequency used on ADS1256
static const float ADC_VREF = 2.5f;       // voltage reference

static const int NUMBER_OF_SAMPLES_FOR_LOADCELL_OFFFSET_ESTIMATION = 1000;
static const float DEFAULT_VARIANCE_ESTIMATE = 0.2f * 0.2f;
static const float LOADCELL_VARIANCE_MIN = 0.001f;
// static const float CONVERSION_FACTOR = LOADCELL_WEIGHT_RATING_KG / (LOADCELL_EXCITATION_V * (LOADCELL_SENSITIVITY_MV_V/1000));

float updatedConversionFactor_f64 = 1.0f;
#define CONVERSION_FACTOR LOADCELL_WEIGHT_RATING_KG / (LOADCELL_EXCITATION_V * (LOADCELL_SENSITIVITY_MV_V / 1000.0f))

uint8_t global_channel0_u8, global_channel1_u8, global_channel2_u8;

ADS1256 &ADC()
{
  static ADS1256 adc(
      ADC_CLOCK_MHZ,
      ADC_VREF,
      /*useresetpin=*/false,
      PIN_DRDY, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS); // RESETPIN is permanently tied to 3.3v

  static bool firstTime = true;
  if (firstTime)
  {
    // ensure the ADS1256 RST pin is held high via internal pull-up
    pinMode(PIN_RST, INPUT_PULLUP);
    digitalWrite(PIN_RST, HIGH);

    Serial.println("Starting ADC");
    adc.initSpi(ADC_CLOCK_MHZ);
    delay(1000);

    Serial.println("ADS: send SDATAC command");
    // adc.sendCommand(ADS1256_CMD_SDATAC);

    // start the ADS1256 with a certain data rate and gain
    adc.begin(ADC_SAMPLE_RATE, ADS1256_GAIN_64, false);

    Serial.println("ADC Started");

    adc.waitDRDY(); // wait for DRDY to go low before changing multiplexer register
    if (fabs(CONVERSION_FACTOR) > 0.01f)
    {
      adc.setConversionFactor(CONVERSION_FACTOR);
    }
    else
    {
      adc.setConversionFactor(1);
    }
    firstTime = false;
  }

  return adc;
}

void LoadCell_ADS1256::setLoadcellRating(uint8_t loadcellRating_u8) const
{
  ADS1256 &adc = ADC();
  float originalConversionFactor_f64 = CONVERSION_FACTOR;

  updatedConversionFactor_f64 = 1.0f;
  if (LOADCELL_WEIGHT_RATING_KG > 0)
  {
    updatedConversionFactor_f64 = 2.0f * ((float)loadcellRating_u8) * (CONVERSION_FACTOR / LOADCELL_WEIGHT_RATING_KG);
  }
  Serial.print("OrigConversionFactor: ");
  Serial.print(originalConversionFactor_f64);
  Serial.print(",     NewConversionFactor:");
  Serial.println(updatedConversionFactor_f64);

  // adc.setConversionFactor( updatedConversionFactor_f64 );
  adc.setConversionFactor(1);
}

LoadCell_ADS1256::LoadCell_ADS1256(uint8_t channel0, uint8_t channel1)
    : _zeroPoint(0.0f), _varianceEstimate(DEFAULT_VARIANCE_ESTIMATE)
{

  global_channel0_u8 = channel0;
  global_channel1_u8 = channel1;
  ADC().setChannel(channel0, channel1); // Set the MUX for differential between ch0 and ch1
  // ADC().setChannel(channel1, channel0);   // Set the MUX for differential between ch1 and ch0
}

float LoadCell_ADS1256::getReadingKg() const
{
  extern volatile float g_lastBrakeKg;
  return g_lastBrakeKg * updatedConversionFactor_f64 - (_zeroPoint + 3.0f * _standardDeviationEstimate);
}

void LoadCell_ADS1256::setZeroPoint()
{
  ADS1256 &adc = ADC();

  Serial.println("ADC: Identify loadcell offset");

  float loadcellOffset = 0.0f; // same name as before
  for (long i = 0; i < NUMBER_OF_SAMPLES_FOR_LOADCELL_OFFFSET_ESTIMATION; ++i)
  {
    adc.waitDRDY();                                                           // block until fresh conversion
    float readingKg = adc.readCurrentChannel() * updatedConversionFactor_f64; // keep existing scale
    loadcellOffset += readingKg;
  }
  loadcellOffset /= NUMBER_OF_SAMPLES_FOR_LOADCELL_OFFFSET_ESTIMATION;

  Serial.print("Offset ");
  Serial.println(loadcellOffset, 10);

  _zeroPoint = loadcellOffset;
}

void LoadCell_ADS1256::estimateVariance()
{
  ADS1256 &adc = ADC();

  Serial.println("ADC: Identify loadcell variance");

  float varNormalizer = 1.0f /
                        (float)(NUMBER_OF_SAMPLES_FOR_LOADCELL_OFFFSET_ESTIMATION - 1); // unchanged name
  float varEstimate = 0.0f;
  for (long i = 0; i < NUMBER_OF_SAMPLES_FOR_LOADCELL_OFFFSET_ESTIMATION; ++i)
  {
    adc.waitDRDY();
    float loadcellReading =
        adc.readCurrentChannel() * updatedConversionFactor_f64 - _zeroPoint; // identical bias removal
    varEstimate += sq(loadcellReading) * varNormalizer;
  }

  _standardDeviationEstimate = sqrtf(varEstimate);

  Serial.println("Variance est.:");
  Serial.println(varEstimate);
  Serial.println("Stddev est.:");
  Serial.println(_standardDeviationEstimate);

  if (varEstimate < LOADCELL_VARIANCE_MIN)
  {
    varEstimate = LOADCELL_VARIANCE_MIN;
  }
  varEstimate *= 9.0f; // keep 3 σ envelope logic
  _varianceEstimate = varEstimate;
}
