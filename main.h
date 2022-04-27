#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
//////////////////

///////////////////
#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640

/** Robot wide IPC bus. */
extern messagebus_t bus;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
