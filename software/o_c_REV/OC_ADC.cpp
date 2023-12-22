#include "OC_ADC.h"
#include "OC_gpio.h"
#include "DMAChannel.h"
#include <algorithm>

namespace OC {

#if defined(__MK20DX256__)
/*static*/ ::ADC ADC::adc_;
#endif
/*static*/ ADC::CalibrationData *ADC::calibration_data_;
/*static*/ uint32_t ADC::raw_[ADC_CHANNEL_LAST];
/*static*/ uint32_t ADC::smoothed_[ADC_CHANNEL_LAST];
#ifdef OC_ADC_ENABLE_DMA_INTERRUPT
/*static*/ volatile bool ADC::ready_;
#endif

#if defined(__MK20DX256__)
constexpr uint16_t ADC::SCA_CHANNEL_ID[DMA_NUM_CH]; // ADCx_SCA register channel numbers
DMAChannel* dma0 = new DMAChannel(false); // dma0 channel, fills adcbuffer_0
DMAChannel* dma1 = new DMAChannel(false); // dma1 channel, updates ADC0_SC1A which holds the channel/pin IDs
DMAMEM static volatile uint16_t __attribute__((aligned(DMA_BUF_SIZE+0))) adcbuffer_0[DMA_BUF_SIZE];

#elif defined(__IMXRT1062__)
#define ADC_SAMPLE_RATE 66000.0f
extern "C" void xbar_connect(unsigned int input, unsigned int output);
DMAChannel dma0(false);
typedef struct {
        uint16_t adc[4];
} adcframe_t;
// sizeof(adc_buffer) must be multiple of 32 byte cache row size
static const int adc_buffer_len = 32;
static DMAMEM __attribute__((aligned(32))) adcframe_t adc_buffer[adc_buffer_len];
static PROGMEM const uint8_t adc2_pin_to_channel[] = {
        7,      // 0/A0  AD_B1_02
        8,      // 1/A1  AD_B1_03
        12,     // 2/A2  AD_B1_07
        11,     // 3/A3  AD_B1_06
        6,      // 4/A4  AD_B1_01
        5,      // 5/A5  AD_B1_00
        15,     // 6/A6  AD_B1_10
        0,      // 7/A7  AD_B1_11
        13,     // 8/A8  AD_B1_08
        14,     // 9/A9  AD_B1_09
        255,    // 10/A10 AD_B0_12 - can't use this pin!
        255,    // 11/A11 AD_B0_13 - can't use this pin!
        3,      // 12/A12 AD_B1_14
        4,      // 13/A13 AD_B1_15
        7,      // 14/A0  AD_B1_02
        8,      // 15/A1  AD_B1_03
        12,     // 16/A2  AD_B1_07
        11,     // 17/A3  AD_B1_06
        6,      // 18/A4  AD_B1_01
        5,      // 19/A5  AD_B1_00
        15,     // 20/A6  AD_B1_10
        0,      // 21/A7  AD_B1_11
        13,     // 22/A8  AD_B1_08
        14,     // 23/A9  AD_B1_09
        255,    // 24/A10 AD_B0_12 - can't use this pin!
        255,    // 25/A11 AD_B0_13 - can't use this pin!
        3,      // 26/A12 AD_B1_14
        4,      // 27/A13 AD_B1_15
#ifdef ARDUINO_TEENSY41
        255,    // 28
        255,    // 29
        255,    // 30
        255,    // 31
        255,    // 32
        255,    // 33
        255,    // 34
        255,    // 35
        255,    // 36
        255,    // 37
        1,      // 38/A14 AD_B1_12
        2,      // 39/A15 AD_B1_13
        9,      // 40/A16 AD_B1_04
        10,     // 41/A17 AD_B1_05
#endif
};
#endif // __IMXRT1062__


#if defined(__MK20DX256__)
/*static*/ void ADC::Init(CalibrationData *calibration_data) {

  adc_.setReference(ADC_REF_3V3);
  adc_.setResolution(kAdcScanResolution);
  adc_.setConversionSpeed(kAdcConversionSpeed);
  adc_.setSamplingSpeed(kAdcSamplingSpeed);
  adc_.setAveraging(kAdcScanAverages);

  calibration_data_ = calibration_data;
  std::fill(raw_, raw_ + ADC_CHANNEL_LAST, 0);
  std::fill(smoothed_, smoothed_ + ADC_CHANNEL_LAST, 0);
  std::fill(adcbuffer_0, adcbuffer_0 + DMA_BUF_SIZE, 0);
  
  adc_.enableDMA();
}

#elif defined(__IMXRT1062__)
/*static*/ void ADC::Init(CalibrationData *calibration_data) {
  calibration_data_ = calibration_data;
  std::fill(raw_, raw_ + ADC_CHANNEL_LAST, 0);
  std::fill(smoothed_, smoothed_ + ADC_CHANNEL_LAST, 0);
}

#endif // __IMXRT1062__

#ifdef OC_ADC_ENABLE_DMA_INTERRUPT
/*static*/ void ADC::DMA_ISR() {
  ADC::ready_ = true;
  dma0->clearInterrupt();
  /* restart DMA in ADC::Scan_DMA() */
}
#endif

/*
 * 
 * DMA/ADC à la https://forum.pjrc.com/threads/30171-Reconfigure-ADC-via-a-DMA-transfer-to-allow-multiple-Channel-Acquisition
 * basically, this sets up two DMA channels and cycles through the four adc mux channels (until the buffer is full), resets, and so on; dma1 advances SCA_CHANNEL_ID
 * somewhat like https://www.nxp.com/docs/en/application-note/AN4590.pdf but w/o the PDB.
 * 
*/

#if defined(__MK20DX256__)
void ADC::Init_DMA() {
  
  dma0->begin(true); // allocate the DMA channel 
  dma0->TCD->SADDR = &ADC0_RA; 
  dma0->TCD->SOFF = 0;
  dma0->TCD->ATTR = 0x101;
  dma0->TCD->NBYTES = 2;
  dma0->TCD->SLAST = 0;
  dma0->TCD->DADDR = &adcbuffer_0[0];
  dma0->TCD->DOFF = 2; 
  dma0->TCD->DLASTSGA = -(2 * DMA_BUF_SIZE);
  dma0->TCD->BITER = DMA_BUF_SIZE;
  dma0->TCD->CITER = DMA_BUF_SIZE; 
  dma0->triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma0->disableOnCompletion();
#ifdef OC_ADC_ENABLE_DMA_INTERRUPT
  dma0->interruptAtCompletion();
  dma0->attachInterrupt(DMA_ISR);
  ready_ = false;
#endif

  dma1->begin(true); // allocate the DMA channel 
  dma1->TCD->SADDR = &ADC::SCA_CHANNEL_ID[0];
  dma1->TCD->SOFF = 2; // source increment each transfer (n bytes)
  dma1->TCD->ATTR = 0x101;
  dma1->TCD->SLAST = - DMA_NUM_CH*2; // num ADC0 samples * 2
  dma1->TCD->BITER = DMA_NUM_CH;
  dma1->TCD->CITER = DMA_NUM_CH;
  dma1->TCD->DADDR = &ADC0_SC1A;
  dma1->TCD->DLASTSGA = 0;
  dma1->TCD->NBYTES = 2;
  dma1->TCD->DOFF = 0;
  dma1->triggerAtTransfersOf(*dma0);
  dma1->triggerAtCompletionOf(*dma0);

  dma0->enable();
  dma1->enable();
} 

#elif defined(__IMXRT1062__)
void ADC::Init_DMA() {
  // Ornament & Crime CV inputs are 19/A5 18/A4 20/A6 17/A3
#ifdef FLIP_180
  const int pin4 = A5;
  const int pin3 = A4;
  const int pin2 = A6;
  const int pin1 = A3;
#else
  const int pin1 = A5;
  const int pin2 = A4;
  const int pin3 = A6;
  const int pin4 = A3;
#endif
  pinMode(pin1, INPUT_DISABLE);
  pinMode(pin2, INPUT_DISABLE);
  pinMode(pin3, INPUT_DISABLE);
  pinMode(pin4, INPUT_DISABLE);

  // configure a timer to trigger ADC
  const int comp1 = ((float)F_BUS_ACTUAL) / (ADC_SAMPLE_RATE) / 2.0f + 0.5f;
  TMR4_ENBL &= ~(1<<3);
  TMR4_SCTRL3 = TMR_SCTRL_OEN | TMR_SCTRL_FORCE;
  TMR4_CSCTRL3 = TMR_CSCTRL_CL1(1) | TMR_CSCTRL_TCF1EN;
  TMR4_CNTR3 = 0;
  TMR4_LOAD3 = 0;
  TMR4_COMP13 = comp1;
  TMR4_CMPLD13 = comp1;
  TMR4_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(3);
  TMR4_DMA3 = TMR_DMA_CMPLD1DE;
  TMR4_CNTR3 = 0;
  TMR4_ENBL |= (1<<3);

  // connect the timer output the ADC_ETC input
  const int trigger = 4; // 0-3 for ADC1, 4-7 for ADC2
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  xbar_connect(XBARA1_IN_QTIMER4_TIMER3, XBARA1_OUT_ADC_ETC_TRIG00 + trigger);

  // turn on ADC_ETC and configure to receive trigger
  if (ADC_ETC_CTRL & (ADC_ETC_CTRL_SOFTRST | ADC_ETC_CTRL_TSC_BYPASS)) {
    ADC_ETC_CTRL = 0; // clears SOFTRST only
    ADC_ETC_CTRL = 0; // clears TSC_BYPASS
  }
  ADC_ETC_CTRL |= ADC_ETC_CTRL_TRIG_ENABLE(1 << trigger) | ADC_ETC_CTRL_DMA_MODE_SEL;
  ADC_ETC_DMA_CTRL |= ADC_ETC_DMA_CTRL_TRIQ_ENABLE(trigger);

  // configure ADC_ETC trigger4 to make four ADC2 measurements
  const int len = 4;
  IMXRT_ADC_ETC.TRIG[trigger].CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(len - 1) |
    ADC_ETC_TRIG_CTRL_TRIG_PRIORITY(7);
  IMXRT_ADC_ETC.TRIG[trigger].CHAIN_1_0 =
    ADC_ETC_TRIG_CHAIN_HWTS0(1) | ADC_ETC_TRIG_CHAIN_HWTS1(1) |
    ADC_ETC_TRIG_CHAIN_CSEL0(adc2_pin_to_channel[pin1]) | ADC_ETC_TRIG_CHAIN_B2B0 |
    ADC_ETC_TRIG_CHAIN_CSEL1(adc2_pin_to_channel[pin2]) | ADC_ETC_TRIG_CHAIN_B2B1;
  IMXRT_ADC_ETC.TRIG[trigger].CHAIN_3_2 =
    ADC_ETC_TRIG_CHAIN_HWTS0(1) | ADC_ETC_TRIG_CHAIN_HWTS1(1) |
    ADC_ETC_TRIG_CHAIN_CSEL0(adc2_pin_to_channel[pin3]) | ADC_ETC_TRIG_CHAIN_B2B0 |
    ADC_ETC_TRIG_CHAIN_CSEL1(adc2_pin_to_channel[pin4]) | ADC_ETC_TRIG_CHAIN_B2B1;

  // set up ADC2 for 12 bit mode, hardware trigger
  //  ADLPC=0, ADHSC=1, 12 bit mode, 40 MHz max ADC clock
  //  ADLPC=0, ADHSC=0, 12 bit mode, 30 MHz max ADC clock
  //  ADLPC=1, ADHSC=0, 12 bit mode, 20 MHz max ADC clock
  uint32_t cfg = ADC_CFG_ADTRG;
  cfg |= ADC_CFG_MODE(2);  // 2 = 12 bits
  cfg |= ADC_CFG_AVGS(0);  // # samples to average
  cfg |= ADC_CFG_ADSTS(2); // sampling time, 0-3
  //cfg |= ADC_CFG_ADLSMP;   // long sample time
  cfg |= ADC_CFG_ADHSC;    // high speed conversion
  //cfg |= ADC_CFG_ADLPC;    // low power
  cfg |= ADC_CFG_ADICLK(0);// 0:ipg, 1=ipg/2, 3=adack (10 or 20 MHz)
  cfg |= ADC_CFG_ADIV(2);  // 0:div1, 1=div2, 2=div4, 3=div8
  ADC2_CFG = cfg;
  //ADC2_GC &= ~ADC_GC_AVGE; // single sample, no averaging
  ADC2_GC |= ADC_GC_AVGE; // use averaging
  ADC2_HC0 = ADC_HC_ADCH(16); // 16 = controlled by ADC_ETC

  // use a DMA channel to capture ADC_ETC output
  dma0.begin();
  dma0.TCD->SADDR = &(IMXRT_ADC_ETC.TRIG[4].RESULT_1_0);
  dma0.TCD->SOFF = 4;
  dma0.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);
  dma0.TCD->NBYTES_MLNO = DMA_TCD_NBYTES_MLOFFYES_NBYTES(8) | // sizeof(adcframe_t) = 8
    DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_MLOFF(-8);
  dma0.TCD->SLAST = -8;
  dma0.TCD->DADDR = adc_buffer;
  dma0.TCD->DOFF = 4;
  dma0.TCD->CITER_ELINKNO = sizeof(adc_buffer) / 8;
  dma0.TCD->DLASTSGA = -sizeof(adc_buffer);
  dma0.TCD->BITER_ELINKNO = sizeof(adc_buffer) / 8;
  dma0.TCD->CSR = 0;
  dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC_ETC);
  dma0.enable();
}
#endif // __IMXRT1062__


#if defined(__MK20DX256__)
/*static*/void FASTRUN ADC::Scan_DMA() {

#ifdef OC_ADC_ENABLE_DMA_INTERRUPT
  if (ADC::ready_)  {
    ADC::ready_ = false;
#else
  if (dma0->complete()) {
    // On Teensy 3.2, this runs every 180us (every 3rd call from 60us timer)
    dma0->clearComplete();
#endif
    dma0->TCD->DADDR = &adcbuffer_0[0];

    /* 
     *  collect  results from adcbuffer_0; as things are, there's DMA_BUF_SIZE = 16 samples in the buffer. 
    */
    uint32_t value;
   
    value = (adcbuffer_0[0] + adcbuffer_0[4] + adcbuffer_0[8] + adcbuffer_0[12]) >> 2; // / 4 = DMA_BUF_SIZE / DMA_NUM_CH
    update<ADC_CHANNEL_1>(value); 

    value = (adcbuffer_0[1] + adcbuffer_0[5] + adcbuffer_0[9] + adcbuffer_0[13]) >> 2;
    update<ADC_CHANNEL_2>(value); 

    value = (adcbuffer_0[2] + adcbuffer_0[6] + adcbuffer_0[10] + adcbuffer_0[14]) >> 2;
    update<ADC_CHANNEL_3>(value); 

    value = (adcbuffer_0[3] + adcbuffer_0[7] + adcbuffer_0[11] + adcbuffer_0[15]) >> 2;
    update<ADC_CHANNEL_4>(value); 

    /* restart */
    dma0->enable();
  }
}

#elif defined(__IMXRT1062__)
static void sum_adc(uint32_t *sum, const adcframe_t *n) {
  sum[0] += n->adc[0];
  sum[1] += n->adc[1];
  sum[2] += n->adc[2];
  sum[3] += n->adc[3];
}

/*static*/void FASTRUN ADC::Scan_DMA() {
  static int old_idx = 0;

  // find the most recently DMA-stored ADC data frame
  const adcframe_t *p = (adcframe_t *)dma0.TCD->DADDR;
  arm_dcache_delete(adc_buffer, sizeof(adc_buffer));
  //asm("dsb");

  uint32_t sum[4] = {0, 0, 0, 0};
  int idx = p - adc_buffer;
  int count = idx - old_idx;
  if (count < 0) count += adc_buffer_len;
  if (count) {
    for (int i=0; i < count ; i++) {
      sum_adc(sum, &adc_buffer[(idx + i) % adc_buffer_len]);
    }

    const int mult = 16;
    update<ADC_CHANNEL_1>(sum[0] * mult / count);
    update<ADC_CHANNEL_2>(sum[1] * mult / count);
    update<ADC_CHANNEL_3>(sum[2] * mult / count);
    update<ADC_CHANNEL_4>(sum[3] * mult / count);

    old_idx = idx;
  }
}

#if defined(ARDUINO_TEENSY41) // Teensy 4.1 - A17 pin identifies PCB hardware
FLASHMEM static
float read_id_voltage() {
  const unsigned int count = 50;
  unsigned int sum=0;
  delayMicroseconds(10);
  for (unsigned int i=0; i < count; i++) {
    sum += analogRead(A17);
  }
  return (float)sum * (3.3f / 1023.0f / (float)count);
}

FLASHMEM
float ADC::Read_ID_Voltage() {
  pinMode(A17, INPUT_PULLUP);
  float volts_pullup = read_id_voltage();
  pinMode(A17, INPUT_PULLDOWN);
  float volts_pulldown = read_id_voltage();
  pinMode(A17, INPUT_DISABLE);
  if (volts_pullup - volts_pulldown > 2.5f) return 0; // pin not connected
  return read_id_voltage();
}
#else // Teensy 4.0
FLASHMEM float ADC::Read_ID_Voltage() { return 0; }
#endif


#endif // __IMXRT1062__


/*static*/ void ADC::CalibratePitch(int32_t c2, int32_t c4) {
  // This is the method used by the Mutable Instruments calibration and
  // extrapolates from two octaves. I guess an alternative would be to get the
  // lowest (-3v) and highest (+6v) and interpolate between them
  // *vague handwaving*
  if (c2 < c4) {
    int32_t scale = (24 * 128 * 4096L) / (c4 - c2);
    calibration_data_->pitch_cv_scale = scale;
  }
}

}; // namespace OC
