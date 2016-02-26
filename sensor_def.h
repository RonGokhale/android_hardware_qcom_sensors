/*
 * =====================================================================================
 * Copyright (C) 2011-2013 Bosch Sensortec GmbH
 *
 *       Filename:  sensor_def.h
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  03/23/2011 05:02:19 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Zhengguang.Guo@bosch-sensortec.com
 *        Company:
 *
 * =====================================================================================
 */

/*************************************************************************************************/
/*  Disclaimer
*
* Common:
* Bosch Sensortec products are developed for the consumer goods industry. They may only be used
* within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
* provided with the express understanding that there is no warranty of fitness for a particular purpose.
* They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
* that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
* Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
* The resale and/or use of products are at the purchasers own risk and his own responsibility. The
* examination of fitness for the intended use is the sole responsibility of the Purchaser.
*
* The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
* incidental, or consequential damages, arising from any product use not covered by the parameters of
* the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
* Sensortec for all costs in connection with such claims.
*
* The purchaser must monitor the market for the purchased products, particularly with regard to
* product safety and inform Bosch Sensortec without delay of all security relevant incidents.
*
* Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
* technical specifications of the product series. They are therefore not intended or fit for resale to third
* parties or for use in end products. Their sole purpose is internal client testing. The testing of an
* engineering sample may in no way replace the testing of a product series. Bosch Sensortec
* assumes no liability for the use of engineering samples. By accepting the engineering samples, the
* Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
* samples.
*
* Special:
* This software module (hereinafter called "Software") and any information on application-sheets
* (hereinafter called "Information") is provided free of charge for the sole purpose to support your
* application work. The Software and Information is subject to the following terms and conditions:
*
* The Software is specifically designed for the exclusive use for Bosch Sensortec products by
* personnel who have special experience and training. Do not use this Software if you do not have the
* proper experience or training.
*
* This Software package is provided `` as is `` and without any expressed or implied warranties,
* including without limitation, the implied warranties of merchantability and fitness for a particular
* purpose.
*
* Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
* of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
* representatives and agents shall not be liable for any direct or indirect damages or injury, except as
* otherwise stipulated in mandatory applicable law.
*
* The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
* responsibility for the consequences of use of such Information nor for any infringement of patents or
* other rights of third parties which may result from its use. No license is granted by implication or
* otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
* subject to change without notice.
*
* It is not allowed to deliver the source code of the Software to any third party without permission of
* Bosch Sensortec.
*/


#ifndef __SENSOR_DEF_H
#define __SENSOR_DEF_H
#include "sensor_data_type.h"

#define SENSOR_HANDLE_ACCELERATION			0
#define SENSOR_HANDLE_MAGNETIC_FIELD			1
#define SENSOR_HANDLE_ORIENTATION			2
#define SENSOR_HANDLE_GYROSCOPE				4
#define SENSOR_HANDLE_LIGHT					5
#define SENSOR_HANDLE_PRESSURE				6
#define SENSOR_HANDLE_TEMPERATURE			7
#define SENSOR_HANDLE_PROXIMITY				8
#define SENSOR_HANDLE_GRAVITY					9
#define SENSOR_HANDLE_LINEAR_ACCELERATION 	10
#define SENSOR_HANDLE_ROTATION_VECTOR 		11

#define SENSOR_HANDLE_ORIENTATION_RAW		(SENSOR_HANDLE_ORIENTATION | 0x80)

#define SET_SENSOR_ACTIVE				0x01
#define SET_SENSOR_DELAY				0x02

#define PATH_DIR_SENSOR_STORAGE "/data/misc/sensor"
#define FIFO_CMD (PATH_DIR_SENSOR_STORAGE "/fifo_cmd")
#define FIFO_DAT (PATH_DIR_SENSOR_STORAGE "/fifo_dat")

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) ((int)(sizeof(arr) / sizeof((arr)[0])))
#endif

#define CHANNEL_PKT_MAGIC_CMD (int)'C'
#define CHANNEL_PKT_MAGIC_DAT (int)'D'

#define SENSOR_ACCURACY_UNRELIABLE	0
#define SENSOR_ACCURACY_LOW			1
#define SENSOR_ACCURACY_MEDIUM		2
#define SENSOR_ACCURACY_HIGH		3

#define SENSOR_MAGIC_A 'a'
#define SENSOR_MAGIC_D 'd'
#define SENSOR_MAGIC_G 'g'
#define SENSOR_MAGIC_L 'l'
#define SENSOR_MAGIC_M 'm'
#define SENSOR_MAGIC_O 'o'
#define SENSOR_MAGIC_P 'p'
#define SENSOR_MAGIC_R 'r'	/* raw orientation */
#define SENSOR_MAGIC_T 't'	/* temperature */

struct exchange {
	int magic;

	union {
		struct {
			short cmd;
			short code;
			short value;

			unsigned char reserved[3];
		} command;

		sensor_data_t data;
	};

	int64_t ts;
};

enum DEV_AVAILABILITY{
	UNAVAILABLE = 0,
	AVAILABLE,
	VIRTUAL
};

#endif
