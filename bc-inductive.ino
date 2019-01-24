#include <eRCaGuy_Timer2_Counter.h>
#include "LowPassFilter.h"


#define CH1_PIN		3
#define CH2_PIN		2

#define CH1		0
#define CH2		1

#define MEASURE_SAMPLE_COUNT	23
#define THRESHOLD_RATIO		4000
#define UNDETECT_THRESHOLD_LEVEL 2/3

struct simple_timer {
	bool active;
	unsigned long scheduled_at;
	unsigned long timeout;
	void (*callback)(void);
};

struct debug_data {
	uint8_t  ch;
	uint16_t period;
	uint16_t bias;
	uint16_t avg;
	uint8_t  presence;
};

#define MAX_DEBUG_COUNT	90
volatile struct debug_data debug_stats[2][MAX_DEBUG_COUNT];
volatile unsigned char debug_count[2] = {0, 0};
volatile unsigned char current_dbg_buf = 0;
volatile char dbg_overflow = 0;

typedef struct simple_timer simple_timer_t;

#define NUM_TIMERS		2
#define PRESENCE_TIMEOUT1_TIMER	0
#define PRESENCE_TIMEOUT2_TIMER	1
#define PRESENCE_TIMEOUT_MS	120000
#define PRESENCE_THRESHOLD	6

simple_timer_t simple_timers[NUM_TIMERS];


const unsigned char led_pins[2] = {A4, A5};

volatile unsigned long	ch_period[2];
volatile uint16_t	threshold[2] = {0, 0};
volatile uint16_t	undetect_threshold[2] = {0, 0};
volatile uint8_t	presence[2] = {0, 0};


#define BIAS_FILTER_K	11
/* init bias filter with low time const first for setup */
LowPassFilter biasFilter[2] = {LowPassFilter(6, 0), LowPassFilter(6, 0)};

LowPassFilter avgFilter[2] = {LowPassFilter(4, 0), LowPassFilter(4, 0)};

void schedule_timer(uint8_t timer_id, unsigned long timeout) {
	simple_timers[timer_id].scheduled_at = millis();
	simple_timers[timer_id].timeout = timeout;
	simple_timers[timer_id].active = true;
}

void stop_timer(uint8_t timer_id) {
	simple_timers[timer_id].active = false;
}

void try_timers(void) {
	int i;
	unsigned long t = millis();

	for (i = 0; i < NUM_TIMERS; i++) {
		if (simple_timers[i].active
				&& (t - simple_timers[i].scheduled_at >= simple_timers[i].timeout)) {
			simple_timers[i].active = false;
			simple_timers[i].callback();
		}
	}
}


volatile unsigned long timer1_overflow_count;
unsigned long timer1_total_count;

ISR(TIMER1_OVF_vect)
{
	timer1_overflow_count++;
}

void timer1_setup(void)
{
	timer1_overflow_count = 0;
	timer1_total_count = 0;

	TCCR1A = 0;
	TCCR1B = 0x01; /* No prescaing */

	TIMSK1 = _BV(TOIE1);
}

unsigned long timer1_get_count(void)
{
	uint8_t SREG_old = SREG;

	cli();
	uint16_t tcnt_save = TCNT1;
	boolean flag_save = bitRead(TIFR1, TOV1);

	if (flag_save) {
		tcnt_save = TCNT1;
		timer1_overflow_count++;
		TIFR1 |= 0x01;
	}

	timer1_total_count = timer1_overflow_count * 0x10000 + tcnt_save;

	SREG = SREG_old;

	return timer1_total_count;
}


inline void measure_channel(unsigned char ch)
{
	static uint8_t ch_count[2] = {0, 0};
	static unsigned long ch_time[2] = {0, 0};

	ch_count[ch]++;

	if (ch_count[ch] == MEASURE_SAMPLE_COUNT) {
		register unsigned long m = timer1_get_count();
		register unsigned long v = m - ch_time[ch];
		unsigned long bias = biasFilter[ch].output();
		unsigned long avg;
		uint16_t thr;

		ch_count[ch] = 0;
		ch_period[ch] = v;
		ch_time[ch] = m;

		avgFilter[ch].input(v);

		avg = avgFilter[ch].output();

		if (!presence[ch])
			thr = threshold[ch];
		else
			thr = undetect_threshold[ch];


		if ((avg < bias - thr) || (avg > bias + thr)) {
			if (presence[ch] < PRESENCE_THRESHOLD) {
				presence[ch]++;

				/* Schedule timeout timer at changing state from no presence to presence */
				if (presence[ch] == PRESENCE_THRESHOLD) {
					schedule_timer(PRESENCE_TIMEOUT1_TIMER + ch, PRESENCE_TIMEOUT_MS);
					digitalWrite(led_pins[ch], LOW);
				}
			}

		} else {
			if (presence[ch]) {
				stop_timer(PRESENCE_TIMEOUT1_TIMER + ch);
				presence[ch] = 0;
				digitalWrite(led_pins[ch], HIGH);
			}

		}

		if (presence[ch] < PRESENCE_THRESHOLD)
			biasFilter[ch].input(v);

		if (ch == 1) {
			unsigned char b = current_dbg_buf;
			unsigned char i = debug_count[b];

			if (i == MAX_DEBUG_COUNT) {
				if (debug_count[1 - b]) {
					dbg_overflow = 1;
				} else {
					b = 1 - b;
					current_dbg_buf = b;
					i = 0;
				}
			}

			if (!dbg_overflow) {
				debug_stats[b][i].ch = ch;
				debug_stats[b][i].period = v;
				debug_stats[b][i].bias = bias;
				debug_stats[b][i].avg = avgFilter[ch].output();
				debug_stats[b][i].presence = presence[ch];

				debug_count[b]++;
			}
		}
	}
}

void ch1_isr()
{
	measure_channel(CH1);
}

void ch2_isr()
{
	measure_channel(CH2);
}

void presenceTimeout1(void)
{
	if (presence[CH1]) {
		cli();
		biasFilter[CH1].setOutput(avgFilter[CH1].output());
		sei();
	}
}

void presenceTimeout2(void)
{
	if (presence[CH2]) {
		cli();
		biasFilter[CH2].setOutput(avgFilter[CH2].output());
		sei();
	}
}


void print_debug(unsigned char b)
{
	int i, ch;

//	Serial.println("Debug stats:");
//	Serial.println("0\t0\t0");

//	for (ch = 0; ch < 2; ch++) {
//		Serial.print("Channel ");
//		Serial.print(ch + 1);
//		Serial.println(":");

		for (i = 0; i < debug_count[b]; i++) {
//			if (debug_stats[i].ch == ch) {
				Serial.print(debug_stats[b][i].period);
				Serial.print('\t');
				Serial.print(debug_stats[b][i].bias);
				Serial.print('\t');
				Serial.print(debug_stats[b][i].avg);
				Serial.print('\t');
				Serial.print(debug_stats[b][i].presence);
				ch = debug_stats[b][i].ch;
				Serial.print('\t');
				Serial.print(threshold[ch]);
				Serial.print('\t');
				Serial.println(undetect_threshold[ch]);
//			}
		}

//	}
}

void setup()
{

//	timer2.setup();
	timer1_setup();

	pinMode(CH1_PIN, INPUT_PULLUP);
	pinMode(CH2_PIN, INPUT_PULLUP);
	pinMode(led_pins[CH1], OUTPUT);
	digitalWrite(led_pins[CH1], HIGH);
	pinMode(led_pins[CH2], OUTPUT);
	digitalWrite(led_pins[CH2], HIGH);

	Serial.begin(115200);

	simple_timers[PRESENCE_TIMEOUT1_TIMER].callback = presenceTimeout1;
	simple_timers[PRESENCE_TIMEOUT1_TIMER].active = false;
	simple_timers[PRESENCE_TIMEOUT2_TIMER].callback = presenceTimeout2;
	simple_timers[PRESENCE_TIMEOUT2_TIMER].active = false;

	attachInterrupt(digitalPinToInterrupt(CH1_PIN), ch1_isr, FALLING);
	attachInterrupt(digitalPinToInterrupt(CH2_PIN), ch2_isr, FALLING);

	delay(300);

	cli();
	avgFilter[CH1].setOutput(ch_period[CH1]);
	avgFilter[CH2].setOutput(ch_period[CH2]);
	sei();

	delay(300);

	cli();
	unsigned long avg;

	avg = avgFilter[CH1].output();
	biasFilter[CH1].setOutput(avg);
	threshold[CH1] = biasFilter[CH1].output() / THRESHOLD_RATIO;
	undetect_threshold[CH1] = avg * UNDETECT_THRESHOLD_LEVEL / THRESHOLD_RATIO;

	avg = avgFilter[CH2].output();
	biasFilter[CH2].setOutput(avg);
	threshold[CH2] = biasFilter[CH2].output() / THRESHOLD_RATIO;
	undetect_threshold[CH2] = avg * UNDETECT_THRESHOLD_LEVEL / THRESHOLD_RATIO;
	sei();

/*	Serial.print("Thresholds: ");
	Serial.print(threshold[0]);
	Serial.print("\t");
	Serial.println(threshold[1]);

*/	delay(1500);

	cli();
	biasFilter[CH1].setK(BIAS_FILTER_K);
	biasFilter[CH2].setK(BIAS_FILTER_K);
	sei();


//	TCCR2A = _BV(WGM21);
//	TCCR2B = _BV(CS22) | _BV(CS20);

	cli();
	current_dbg_buf = 0;
	dbg_overflow = 0;
	debug_count[0] = 0;
	debug_count[1] = 0;
	sei();
}


void loop()
{
	static unsigned char b = 0;

	while (debug_count[b] < MAX_DEBUG_COUNT);

	print_debug(b);

	cli();
	debug_count[b] = 0;
	sei();
	b = 1 - b;

	if (dbg_overflow) {
		cli();
		dbg_overflow = 0;
		sei();
		Serial.println("Debug OVERFLOW");
	}

	try_timers();
//	Serial.flush();
}
