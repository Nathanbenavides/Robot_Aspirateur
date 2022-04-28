#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;
static uint16_t pos_line = 0;
static uint16_t width_line = 0;

uint8_t get_mean(uint8_t* image);
void image_width_pos(uint8_t* image);

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 200, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);

        	//static uint16_t time = 0;
        	//chprintf((BaseSequentialStream *)&SD3, "time = %d \n", chVTGetSystemTime()- time);
        	//time = chVTGetSystemTime();

		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(uint16_t i = 0; i < PO8030_MAX_WIDTH*2; i+=2){
			image[i/2] = img_buff_ptr[i+1] & 31;
		}

		SendUint8ToComputer(image, PO8030_MAX_WIDTH);

		image_width_pos(image);

		distance_cm = 1350.00/width_line;

//		chprintf((BaseSequentialStream *)&SDU1, "Width = %d Pos = %d dist = %.2f\r\n", width_line, pos_line, distance_cm);
    }
}

uint8_t get_mean(uint8_t* image){
	uint8_t max = image[0];
	uint8_t min = image[0];
	for(uint16_t i = 1; i < PO8030_MAX_WIDTH; i++){
		max = ( (max > image[i]) ? max : image[i] );
		min = ( (min < image[i]) ? min : image[i] );
	}

	return (max-min)/3;
}

void image_width_pos(uint8_t* image){
	uint16_t width = 0;
	uint16_t somme_pos = 0;
	uint8_t mean = get_mean(image);

	for(uint16_t i = 0; i < PO8030_MAX_WIDTH; i++){
		if(image[i]<mean){
			width++;
			somme_pos += i;
		}
	}

	width_line = width;
	pos_line = somme_pos/width;
}


float get_distance_cm(void){

	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
