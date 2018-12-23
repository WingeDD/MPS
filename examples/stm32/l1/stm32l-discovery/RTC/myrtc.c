#define STM32L1
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/l1/lcd.h>
#include <libopencm3/stm32/l1/rtc.h>
#include <libopencm3/stm32/usart.h>

static void set_rtc_clock(void) {
	rcc_periph_clock_enable(RCC_PWR);
	pwr_disable_backup_domain_write_protect();
	rcc_osc_on(RCC_LSE);
	rcc_rtc_select_clock(RCC_CSR_RTCSEL_LSE);
	pwr_enable_backup_domain_write_protect();
	rcc_wait_for_osc_ready(RCC_LSE);
}

static void init_gpio(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	/*rcc_peripheral_enable_clock(&RCC_AHBLPENR, RCC_AHBLPENR_GPIOALPEN
						 | RCC_AHBLPENR_GPIOBLPEN
						 | RCC_AHBLPENR_GPIOCLPEN);*/
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO1  | GPIO2  | GPIO3  | GPIO8
		      | GPIO9  | GPIO10 | GPIO15);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO3  | GPIO4  | GPIO5  | GPIO6
		      | GPIO7  | GPIO8  | GPIO9  | GPIO10
		      | GPIO11 | GPIO12 | GPIO13 | GPIO14
		      | GPIO15);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO0  | GPIO1  | GPIO2  | GPIO3
		      | GPIO6  | GPIO7  | GPIO8  | GPIO9
		      | GPIO10 | GPIO11);

	gpio_set_af(GPIOA, GPIO_AF11, GPIO1  | GPIO2  | GPIO3
		  | GPIO8  | GPIO9  | GPIO10 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF11, GPIO3  | GPIO4  | GPIO5
		  | GPIO8  | GPIO9  | GPIO10 | GPIO11 | GPIO12
		  | GPIO13 | GPIO14 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF7, GPIO6 | GPIO7);
	gpio_set_af(GPIOC, GPIO_AF11, GPIO0  | GPIO1  | GPIO2
		  | GPIO3  | GPIO6  | GPIO7  | GPIO8  | GPIO9
		  | GPIO10 | GPIO11);
}

static void init_lcd(void) {
	rcc_periph_clock_enable(RCC_LCD);
	lcd_enable_segment_multiplexing();
	lcd_set_duty(LCD_CR_DUTY_1_4);
	lcd_set_bias(LCD_CR_BIAS_1_3);
	lcd_set_refresh_frequency (100);
	lcd_set_contrast(LCD_FCR_CC_4);
	lcd_enable();
	while(!lcd_is_enabled());
	while(!lcd_is_step_up_ready());
}

static void init_rtc(void) {
	rcc_periph_clock_enable(_REG_BIT(0x34, 22)); //RCC_RTC
	pwr_disable_backup_domain_write_protect();
	rtc_unlock();
	RTC_ISR |= RTC_ISR_INIT;
	while(!(RTC_ISR & RTC_ISR_INITF));
	RTC_TR = 0x00; //0x00235930
	RTC_DR = 0x00708101;
	RTC_ISR &= ~(RTC_ISR_INIT);
	rtc_lock();
	pwr_enable_backup_domain_write_protect();
	//rtc_wait_for_synchro();
}

static void init_usart(void) {
	rcc_periph_clock_enable(RCC_USART1);
	usart_set_baudrate(USART1, 38400);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}

static void clear_lcd_ram(void)
{
	LCD_RAM_COM0 = 0;
	LCD_RAM_COM1 = 0;
	LCD_RAM_COM2 = 0;
	LCD_RAM_COM3 = 0;
}

/*	LCD MAPPING:
	    A
     _  ----------
COL |_| |\   |J  /|
       F| H  |  K |B
     _  |  \ | /  |
COL |_| --G-- --M--
        |   /| \  |
       E|  Q |  N |C
     _  | /  |P  \|
DP  |_| -----------
	    D

`mask' corresponds to bits in lexicographic order: mask & 1 == A, mask & 2 == B,
and so on.
 */
static void write_mask_to_lcd_ram (int position, uint16_t mask, int clear_before)
{
	/* Every pixel of character at position can be accessed
	   as LCD_RAM_COMx & (1 << Px) */
	int P1,P2,P3,P4;
	if (position < 2) P1 = 2*position;
	else P1 = 2*position+4;
	
	if (position == 1) P2 = P1+5;
	else P2 = P1+1;
	
	if (position < 3) P3 = (23-2*position)+6;
	else P3 = (23-2*position)+4;
	
	if (position == 5) {
		P4 = P3;
		P3 -= 1;
	} else {
		P4 = P3 - 1;
	}

	uint32_t COM0,COM1,COM2,COM3;
	COM0 = LCD_RAM_COM0;
	COM1 = LCD_RAM_COM1;
	COM2 = LCD_RAM_COM2;
	COM3 = LCD_RAM_COM3;
	
	if (clear_before) {
		COM0&= ~(1<<P1 | 1<<P2 | 1<<P3 | 1<<P4);
		COM1&= ~(1<<P1 | 1<<P2 | 1<<P3 | 1<<P4);
		COM2&= ~(1<<P1 | 1<<P2 | 1<<P3 | 1<<P4);
		COM3&= ~(1<<P1 | 1<<P2 | 1<<P3 | 1<<P4);
	}
	
	COM0 |= ((mask >> 0x1) & 1) << P4 | ((mask >> 0x4) & 1) << P1
	      | ((mask >> 0x6) & 1) << P3 | ((mask >> 0xA) & 1) << P2;
	COM1 |= ((mask >> 0x0) & 1) << P4 | ((mask >> 0x2) & 1) << P2
	      | ((mask >> 0x3) & 1) << P1 | ((mask >> 0x5) & 1) << P3;
	COM3 |= ((mask >> 0x7) & 1) << P3 | ((mask >> 0x8) & 1) << P4
	      | ((mask >> 0xB) & 1) << P1 | ((mask >> 0xE) & 1) << P2;
	COM2 |= ((mask >> 0x9) & 1) << P4 | ((mask >> 0xC) & 1) << P1
	      | ((mask >> 0xD) & 1) << P3 | ((mask >> 0xF) & 1) << P2;

	LCD_RAM_COM0 = COM0;
	LCD_RAM_COM1 = COM1;
	LCD_RAM_COM2 = COM2;
	LCD_RAM_COM3 = COM3;
}

static void write_char_to_lcd_ram (int position, uint8_t symbol, bool clear_before)
{
	uint16_t from_ascii[0x60] = {
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /*         !       "       #       $      %        &       ' */
	  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /* (       )       *       +       ,       -       .       / */
	  0x0000, 0x0000, 0x3FC0, 0x1540, 0x0000, 0x0440, 0x4000, 0x2200,
	  /* 0       1       2       3       4       5       6       7 */
	  0x003F, 0x0006, 0x045B, 0x044F, 0x0466, 0x046D, 0x047D, 0x2201,
	  /* 8       9       :       ;       <       =       >       ? */
	  0x047F, 0x046F, 0x8000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	  /* @       A       B       C       D       E       F       G */
	  0x0000, 0x0477, 0x047C, 0x0039, 0x045E, 0x0479, 0x0471, 0x043D,
	  /* H       I       J       K       L       M       N       O */
	  0x0476, 0x1109, 0x001E, 0x1B00, 0x0038, 0x02B6, 0x08B6, 0x003F,
	  /* P       Q       R       S       T       U       V       W */
	  0x0473, 0x0467, 0x0C73, 0x046D, 0x1101, 0x003E, 0x0886, 0x2836,
	  /* X       Y       Z       [       \       ]       ^       _ */
	  0x2A80, 0x1280, 0x2209, 0x0000, 0x0880, 0x0000, 0x0000, 0x0008
	};
	if (symbol > 0x60) return; // masks not defined. Nothing to display
	
	write_mask_to_lcd_ram (position, from_ascii[symbol], clear_before);
}

static void lcd_display(char *buffer) {
	int i = 0;
	int j = 0;
	char c = 0;
	while((c = buffer[i]) != '\0') {
		if (c == ':' || c == '.') {
			j++;
			write_char_to_lcd_ram(i - j, c, false);
		}
		else {
			write_char_to_lcd_ram(i - j, c, true);
		}
		i++;
	}
	lcd_update();
}

static int is_digit(char c) {
	for(int i = '0'; i <= '9'; i++) {
		if(c == i) {
			return 1;
		}
	}
	return 0;
}

static int rtc_check_time(char* time, char* date, char time_separator, char date_separator) {
	int check = 1;
	for(int i = 0; i / 2 < 6; i += 3) {
		check = check && is_digit(time[i / 2]);
		check = check && is_digit(date[i / 2]);
	}
	int h = 10 * (time[0] - '0') + (time[1] - '0');
	int min = 10 * (time[0] - '0') + (time[1] - '0');
	int s = 10 * (time[0] - '0') + (time[1] - '0');
	check = check && (h < 24) && (min < 60) && (s < 60);
	check = check && (time[2] == time_separator) && (time[5] == time_separator);

	int d = 10 * (date[0] - '0') + (date[1] - '0');
	int m = 10 * (date[3] - '0') + (date[4] - '0');
	int y = 10 * (date[6] - '0') + (date[7] - '0');
	switch (m) {
		case 1:
		case 3:
		case 5:
		case 7:
		case 8:
		case 10:
		case 12:
			check = check && (d <= 31);
			break;
		case 4:
		case 6:
		case 9:
		case 11:
			check = check && (d <= 30);
			break;
		case 2:
			check = check && ((d <= 28 && y % 4 != 0) || (d <= 29 && y % 4 == 0));
			break;
		default:
			check = 0;
			break;
	}
	
	check = check && (d != 0) && (m != 0);
	check = check && (date[2] == date_separator) && (date[5] == date_separator);
	return check;
}

static void rtc_set_time(char* time, char* date) { //переписывает резервные значения
	pwr_disable_backup_domain_write_protect();
	rtc_unlock();
	RTC_ISR |= RTC_ISR_INIT;

	while(!(RTC_ISR & RTC_ISR_INITF));
	int xtime = 0x00;
	int xdate = 0x00;
	int k = 0;
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 2; j++) {
			int l = 3 * i + j;
			int m = 1 + 3 * i - j;
			xtime += (int)(time[l] - '0') << ((5 - k) * 4);
			xdate += (int)(date[m] - '0') << (k * 4);
			k++;
		}
	}
	int d = date[0] + date[1];
	int m = (date[3] + date[4] + 10) % 12;
	int y = date[6] + date[7] - (m <= 10)? 0 : 1;
	int c = 20 - (m <= 10 || y != 0)? 0 : 1;
	int w = d + (13 * m - 1) / 5 + y + y / 4 + c / 4 - 2 * c;
	int n = w % 7;
	n = (n == 0)? 7 : n;
	xdate |= n << 13;
	RTC_TR = xtime;
	RTC_DR = xdate;

	RTC_ISR &= ~(RTC_ISR_INIT);
	rtc_lock();
	pwr_enable_backup_domain_write_protect();
}

static void rtc_get_time(char* time, char* date) { //считывает лишние(резервные) значения
	int xtime = RTC_TR;
	int xdate = RTC_DR;
	int k = 0;
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 2; j++) {
			int l = 3 * i + j;
			int m = 1 + 3 * i - j;
			time[l] = ((char)(xtime >> ((5 - k) * 4)) & 0x0F) + '0';
			date[m] = ((char)(xdate >> (k * 4)) & 0x0F) + '0';
			k++;
		}
	}
	time[2] = time[5] = ':';
	date[2] = date[5] = '.';
	date[3] = ((char)(xdate >> 12) & 0x01) + '0';
}

static void usart_send_string(char *buffer) {
	int i = 0;
	char c = 0;
	while ((c = buffer[i]) != '\0') {
		usart_send_blocking(USART1, c);
		i++;
	}
	usart_send_blocking(USART1, '\n');
}

static void usart_recv_string(char *buffer) {
	int i = 0;
	char c = 0;
	while ((c = usart_recv_blocking(USART1)) != '\n') {
		usart_send_blocking(USART1, c);
		buffer[i] = c;
		i++;
	}
	usart_send_blocking(USART1, '\n');
	buffer[i] = '\0';
}

int main(void) { //время в неправильном формате
	set_rtc_clock();
	init_gpio();
	init_lcd();
	init_rtc();
	init_usart();

	char time[9];
	char date[9];
	do {
		usart_recv_string(time);
		usart_recv_string(date);
	} while(!rtc_check_time(time, date, ':', '.'));
	char string[10] = "QWERTY";
	lcd_display(string);
	rtc_set_time(time, date);
	while(1) {
		rtc_get_time(time, date);
		if(GPIOA_IDR & 0x01) {
			usart_send_string(date);
			lcd_display(date);
		}
		else {
			usart_send_string(time);
			lcd_display(time);
		}
	}
	return 0;
}
