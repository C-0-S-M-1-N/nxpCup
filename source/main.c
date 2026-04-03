#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_pwm.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "hbridge.h"
#include "pixy.h"
#include "fsl_common.h"
#include "Config.h"
#include "servo.h"
#include "esc.h"
#include <math.h>

#define MAX_VECTORS 10
#define PI 3.14159265358979323

// constanta de low pass filter ca sa faca steering doar la curbele proeminente ce nu se modifica constant
const double LowPassFilterCoef = 0.7; // mereu subunitar, cat % sa conteze rezultatul nou
const double SpanCameraOffset = 0; // de modificat ca sa mearga masinuta drep in fata pe teren drept
const double proportionalLengthToSpeedCoefficient = 0.3;
const double proportionalSteerToSpeedCoefficient = 0; // la 0 masinuta se misca constant
const double MAX_SPEED = 200;

// cod pentru test camera pixy
void TestCamera(pixy_t* cam){
	uint16_t vectors[MAX_VECTORS * 4];
	size_t num_vectors;
	int angle = 0;
	while(1){
		angle = 0;
		if (pixy_get_vectors(cam, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
			double angle = 0.0;
			for (size_t i = 0; i < num_vectors; i++) {
				uint16_t x0 = vectors[4*i + 0];
				uint16_t y0 = vectors[4*i + 1];
				uint16_t x1 = vectors[4*i + 2];
				uint16_t y1 = vectors[4*i + 3];
				PRINTF("  [%2u] (%u,%u)->(%u,%u)\r\n", (unsigned)i, x0, y0, x1, y1);
				double m = ((double)x0-(double)x1) / ((double)y0-(double)y1);
				angle = angle * (1 - LowPassFilterCoef) + m * LowPassFilterCoef;
			}
		}
		PRINTF(" Steer %d\r\n", angle);
		Steer(angle);
	}
}

void MainUpdateLoop(pixy_t* cam){
	uint16_t vectors[MAX_VECTORS * 4];
	size_t num_vectors;
	int angle = 0;
	int speed = -90, speed1, speed2; // max supported speed
	double angleInRadians = 0;
	while(1){
			angle = 10; // pentru a tine masinuta cat mai pe margine in caz ca nu vede drumul
			speed = -60; // daca nu vede nici o linie atunci merge fix inainte pana gaseste una
			angleInRadians = 10 * PI / 180;
			if (pixy_get_vectors(cam, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
				uint16_t x0, y0, x1, y1;
				x0 = vectors[0]; // vrem doar prima detectie care e si cea mai buna. Vezi Pixy2 docs
				x1 = vectors[0];
				y0 = vectors[0];
				y1 = vectors[0];
				//double m = double(x0 - x1) / double(y0 - y1);
				double vectorRotation = (atan2(y0 - y1, x0 - x1) * 180) / PI;
				double normalRotation = vectorRotation + SpanCameraOffset;
				double vectorLen = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
				angle = angle * (1 - LowPassFilterCoef) + LowPassFilterCoef * normalRotation;
				double speed_tmp = vectorLen * proportionalLengthToSpeedCoefficient - proportionalSteerToSpeedCoefficient * angle;

				if(speed_tmp > MAX_SPEED) speed_tmp = MAX_SPEED;
				// normalize speed in 0, 180;
				speed = speed_tmp / MAX_SPEED * 180;

				PRINTF(" speed: %d %f \r\n", speed, speed_tmp);
				PRINTF(" steer: %d \r\n\r\n", angle);
				angleInRadians = angle * PI / 180;
		}
		int speed1 = speed - speed*sin(angleInRadians);
		int speed2 = speed + speed*sin(angleInRadians);
		Steer(angle);
		HbridgeSpeed(&g_hbridge, speed1 - 100, speed2 - 100);

	}
}

void testMotors(){
	while(1){
		int speed = -100;
		for(int speed = -100; speed ++ < 100;){
			for(int delay = 800000; delay--;);
			HbridgleSpeed(speed);
		}
	}
}

int main(void)
{

    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);

    // set servo to 0
//    Steer(0);
//    for(;;);



    //HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);
    //HbridgeSpeed(&g_hbridge, 0, 0);
    //return 0;

//    TestServo();
//    return 0;
    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 255, 255, 255);

    TestCamera(&cam1);
    while (1)
    {
    	MainUpdateLoop(&cam1);
    }
}
