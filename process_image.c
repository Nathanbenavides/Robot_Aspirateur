#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#include <selector.h>

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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


static THD_WORKING_AREA(waProcessImage, 3072);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    static int select = 0;
	uint8_t *img_buff_ptr;
	uint8_t imageR[IMAGE_BUFFER_SIZE] = {0};
	uint8_t imageG[IMAGE_BUFFER_SIZE] = {0};
	uint8_t imageB[IMAGE_BUFFER_SIZE] = {0};

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);

		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		if(get_selector() == select){

		chprintf((BaseSequentialStream *)&SDU1, "image start\r\n");

		for(uint16_t j = 0; j < 480; j++){
			chprintf((BaseSequentialStream *)&SDU1, "{ ");
			uint16_t k = PO8030_MAX_WIDTH*2*j;

			for(uint16_t i = 0; i < PO8030_MAX_WIDTH*2; i+=2){

				imageR[i/2] = (img_buff_ptr[i+k] & 0xF8)>>3;
				imageG[i/2] = ((img_buff_ptr[i+k] & 0x7)<<3) + ((img_buff_ptr[i+k+1] & 0xE0)>>5);
				imageB[i/2] = img_buff_ptr[i+1+k] & 31;


				chprintf((BaseSequentialStream *)&SDU1, "{%d , %d , %d}, ", imageR[i/2], imageG[i/2], imageB[i/2]);
			}
			chprintf((BaseSequentialStream *)&SDU1, " }, ");
    	}
		chprintf((BaseSequentialStream *)&SDU1, "\r\n");

		select = get_selector();
		}
    }
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
