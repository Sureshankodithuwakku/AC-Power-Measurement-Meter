#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* ===================== USER CONFIG ===================== */
#define VREF_MV      5000UL     // measure your AVCC (e.g., 4980)
#define LCD_ADDR     0x27
#define TWI_FREQ     100000UL

// ADC channels
#define ADC_CH_VOLT  3          // PC3 = ADC3 (Voltage input)
#define ADC_CH_CURR  2          // PC2 = ADC2 (Current input)

// Sampling (small to save RAM)
#define MAINS_HZ            50
#define SAMPLES_PER_CYCLE   64
#define WINDOWS_CYC         2
#define SAMPLE_US           (1000000UL/(MAINS_HZ*SAMPLES_PER_CYCLE))
#define N   (SAMPLES_PER_CYCLE * WINDOWS_CYC)

// Calibration
#define V_PER_VADC   100.0f     // mains volts per ADC volt
#define A_PER_VADC   30.0f      // amps per ADC volt RMS

/* ===================== I2C / LCD ===================== */
static void i2c_init(void){ TWSR=0; TWBR=(uint8_t)((F_CPU/TWI_FREQ-16)/2); }
static void i2c_start(uint8_t addr){ TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); TWDR=addr; TWCR=(1<<TWINT)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); }
static void i2c_write(uint8_t d){ TWDR=d; TWCR=(1<<TWINT)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); }
static void i2c_stop(void){ TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO); }

#define PCF8574_W ((LCD_ADDR<<1)|0)
#define LCD_RS_BIT 0
#define LCD_RW_BIT 1
#define LCD_EN_BIT 2
#define LCD_BL_BIT 3
#define LCD_D4_BIT 4
#define LCD_D5_BIT 5
#define LCD_D6_BIT 6
#define LCD_D7_BIT 7
static uint8_t lcd_bl=(1<<LCD_BL_BIT);

static void lcd_i2c_write(uint8_t v){ i2c_start(PCF8574_W); i2c_write(v); i2c_stop(); }
static void lcd_pulse_en(uint8_t d){ lcd_i2c_write(d|(1<<LCD_EN_BIT)); _delay_us(1); lcd_i2c_write(d&~(1<<LCD_EN_BIT)); _delay_us(50); }
static void lcd_write4(uint8_t hi,uint8_t rs){
  uint8_t d=lcd_bl|(rs?(1<<LCD_RS_BIT):0);
  if(hi&0x10)d|=(1<<LCD_D4_BIT); if(hi&0x20)d|=(1<<LCD_D5_BIT);
  if(hi&0x40)d|=(1<<LCD_D6_BIT); if(hi&0x80)d|=(1<<LCD_D7_BIT);
  lcd_pulse_en(d);
}
static void lcd_write8(uint8_t v,uint8_t rs){ lcd_write4(v&0xF0,rs); lcd_write4((v<<4)&0xF0,rs); }
static void lcd_cmd(uint8_t c){ lcd_write8(c,0); if(c==0x01||c==0x02)_delay_ms(2); }
static void lcd_data(uint8_t c){ lcd_write8(c,1); }
static void lcd_backlight_on(uint8_t on){ lcd_bl=on?(1<<LCD_BL_BIT):0; lcd_i2c_write(lcd_bl); }
static void lcd_init(void){
  i2c_init(); _delay_ms(40); lcd_backlight_on(1);
  lcd_write4(0x30,0); _delay_ms(5); lcd_write4(0x30,0); _delay_us(150);
  lcd_write4(0x30,0); _delay_us(150); lcd_write4(0x20,0); _delay_us(150);
  lcd_cmd(0x28); lcd_cmd(0x08); lcd_cmd(0x01); _delay_ms(2); lcd_cmd(0x06); lcd_cmd(0x0C);
}
static void lcd_set_cursor(uint8_t col,uint8_t row){ static const uint8_t row_addr[]={0x00,0x40,0x14,0x54}; lcd_cmd(0x80|(row_addr[row]+col)); }
static void lcd_print(const char*s){ while(*s)lcd_data(*s++); }
static void print_float_1dp(float v){ long iv=(long)(v*10+(v>=0?0.5f:-0.5f)); long w=iv/10; long f=labs(iv%10); char b[12]; ltoa(w,b,10); lcd_print(b); lcd_data('.'); lcd_data('0'+f); }
static void print_float_2dp(float v){ long iv=(long)(v*100+(v>=0?0.5f:-0.5f)); long w=iv/100; long f=labs(iv%100); char b[12]; ltoa(w,b,10); lcd_print(b); lcd_data('.'); lcd_data('0'+(f/10)); lcd_data('0'+(f%10)); }

/* ===================== ADC ===================== */
static void adc_init_avcc(void){ ADMUX=(1<<REFS0); ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); }
static inline uint16_t adc_read_once(uint8_t ch){ ADMUX=(ADMUX&0xF0)|(ch&0x0F); ADCSRA|=(1<<ADSC); while(ADCSRA&(1<<ADSC)); return ADC; }

/* ===================== Buffers ===================== */
static int16_t v_raw[N];
static int16_t i_raw[N];

/* ===================== Acquire samples ===================== */
static void acquire(void){
  (void)adc_read_once(ADC_CH_VOLT); (void)adc_read_once(ADC_CH_CURR);
  for(uint16_t n=0;n<N;n++){
    v_raw[n]=adc_read_once(ADC_CH_VOLT);
    i_raw[n]=adc_read_once(ADC_CH_CURR);
    _delay_us(SAMPLE_US>30?(SAMPLE_US-30):0);
  }
}

/* ===================== Compute V,I,P ===================== */
static void compute(float*Vrms,float*Irms,float*P){
  const float k=(float)VREF_MV/1023.0f/1000.0f; // counts→V at ADC pin
  // Means
  uint32_t sv=0,si=0;
  for(uint16_t n=0;n<N;n++){ sv+=v_raw[n]; si+=i_raw[n]; }
  float mv=(float)sv/N; float mi=(float)si/N;
  // Stats
  double s2v=0,s2i=0,pwr=0;
  for(uint16_t n=0;n<N;n++){
    float vv=((float)v_raw[n]-mv)*k;
    float ii=((float)i_raw[n]-mi)*k;
    s2v+=(double)vv*vv;
    s2i+=(double)ii*ii;
    pwr+=(double)vv*ii;
  }
  float v_rms_adc=sqrtf((float)(s2v/N));
  float i_rms_adc=sqrtf((float)(s2i/N));
  *Vrms=v_rms_adc*V_PER_VADC;
  *Irms=i_rms_adc*A_PER_VADC;
  P=(pwr/N)(V_PER_VADC*A_PER_VADC);
}

/* ===================== MAIN ===================== */
int main(void){
  lcd_init(); adc_init_avcc();
  while(1){
    acquire();
    float Vrms=0,Irms=0,P=0; compute(&Vrms,&Irms,&P);
    lcd_cmd(0x01);
    lcd_set_cursor(0,0); lcd_print("V="); print_float_1dp(Vrms); lcd_print("V ");
    lcd_set_cursor(0,1); lcd_print("I="); print_float_2dp(Irms); lcd_print("A ");
    lcd_set_cursor(9,1); lcd_print("P="); print_float_1dp(P); lcd_print("W");
    _delay_ms(600);
  }
}