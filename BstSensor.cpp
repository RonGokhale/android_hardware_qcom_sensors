/*
 * Copyright (C) 2008-2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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


#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>


#include <linux/fs.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>


#include <cutils/log.h>

#include "sensors.h"

#include "BstSensor.h"

#define GET_HANDLES_TRY_NUM         20

const int BstSensor::s_tab_id2handle[BST_SENSOR_NUM_MAX] = {SENSORS_ACCELERATION_HANDLER, SENSORS_MAGNETIC_FIELD_HANDLER, SENSORS_ORIENTATION_HANDLER, SENSORS_LIGHT_HANDLER, SENSORS_PROXIMITY_HANDLER, SENSORS_GYROSCOPE_HANDLER};


const int BstSensor::s_tab_handle2id[BST_SENSOR_NUM_MAX] = {ID_A, ID_M, ID_O,ID_L, ID_P};

int     numOfHandles = 0;

int BstSensor::initStorage()
{
	char *path = NULL;
	int err = 0;
	struct stat st;
	int retry = 0;

	path = (char *)PATH_DIR_SENSOR_STORAGE;
	err = access(path, F_OK);
	if (!err) {
		st.st_mode = ~S_IFDIR;
		err = stat(path, &st);
		if (!S_ISDIR(st.st_mode)) {
			ALOGE("<BST>" "a file of the same name exists");
			/* mark error explicitly for creating the dir */
			err = -ENOTDIR;
		}
	}

	while (0 !=err && retry < 4) {
		ALOGE("<BST>" "create path of storage, err: %d, retry count %d", err, retry);
		unlink(path);
		err = mkdir(path, 0775);
		if (err) {
			ALOGE("<BST>" "error creating storage dir");
		}
		retry++;
		usleep(1000);
	}

	return err;
}

int BstSensor::initIPC()
{
	int err = 0;
	char *filename;

	err = initStorage();
	if (err) {
		ALOGE("initStorage return %d", err);
		return err;
	}

	filename = (char *)FIFO_CMD;
	err = access(filename, F_OK | R_OK | W_OK);
	if (err) {
		ALOGE("<BST> " "can not access %s", filename);
		return err;
	}

	mCmdFd = open(filename, O_RDWR);
	if (mCmdFd < 0) {
		ALOGE("<BST> " "error openning file: %s", filename);
		mCmdFd = -abs(errno);
		err = mCmdFd;
		return err;
	}

	filename = (char *)FIFO_DAT;
	err = access(filename, F_OK | R_OK | W_OK);
	if (err) {
		ALOGE("<BST> " "can not access %s", filename);
		return err;
	}

	data_fd = open(filename, O_RDWR);
	if (data_fd < 0) {
		ALOGE("<BST> " "error openning file: %s", filename);
		err = data_fd;
		return err;
	}

	data_name = filename;

	ALOGE("<BST> " "mCmdFd: %d data_fd: %d", mCmdFd, data_fd);

	return err;
}


BstSensor::BstSensor()
	: SensorBase(BST_DEVICE_NAME, "null"),
	mEnabled(0)
{
	struct exchange cmd;
	int             i = 0;
	int             ret = 0;

	do {
		if (i ++ > GET_HANDLES_TRY_NUM)
		{
			ALOGE("initIPC failed:%d", ret);
			return;
		}
		ret = initIPC();
		if (!ret)
		{
			break;
		}
		sleep(1);
	} while(ret);

	/* if handles already initialized */
	if (numOfHandles > 0)
	{
		return;
	}
}


BstSensor::~BstSensor()
{
	/* mCmdFd is added by BstSensor, while
	 * data_fd is added by SensorBase
	 * so here, only close mCmdFd */
	if (mCmdFd >= 0) {
		close(mCmdFd);
	}
}

int BstSensor::id2handle(int32_t id)
{
	if ( (id < 0) || (id >= BST_SENSOR_NUM_MAX) ) {
		ALOGE("Invalid ID");
		return -1;
	} else {
		return BstSensor::s_tab_id2handle[id];
	}
}


int BstSensor::handle2id(int32_t handle)
{
	if ( (handle - 1 < 0) || (handle - 1 >= BST_SENSOR_NUM_MAX) ) {
		ALOGE("<BST> " "Invalid handle %d", handle);
		return -1;
	} else {
		return BstSensor::s_tab_handle2id[handle - 1];
	}
}


int BstSensor::enable(int32_t id, int enable)
{
	int err = 0;
	int handle;
	struct exchange cmd;
	int pos = (int)id;

	const struct sensor_t *s;

	handle = BstSensor::id2handle(id);
	if (-1 == handle) {
		return -EINVAL;
	}

	if (mCmdFd < 0) {
		ALOGE("<BST> " "cannot tx cmd: %s",
				(char *)strerror(-mCmdFd));
		return mCmdFd;
	}

	enable = !!enable;

	s = BstSensorInfo::getSensor(id);
	ALOGI("<BST> " "%s sensor <%s>",
			enable ? "enable" : "disable",
			(s != NULL) ? s->name : "unknown");

	cmd.magic = CHANNEL_PKT_MAGIC_CMD;
	cmd.command.cmd = SET_SENSOR_ACTIVE;
	cmd.command.code = handle;
	cmd.command.value = enable;
	cmd.ts = 0;
	err = write(mCmdFd, &cmd, sizeof(cmd));
	err = err < (int)sizeof(cmd) ? -1 : 0;

	if (!err) {
		if (enable) {
			mEnabled |= (1 << pos);
		} else {
			mEnabled &= ~(1 << pos);
		}
	}

	return err;
}


int BstSensor::setDelay(int32_t id, int64_t ns)
{
	int err = 0;
	int handle;
	struct exchange cmd;
	int pos = (int)id;
	const struct sensor_t *s;

	handle = BstSensor::id2handle(id);
	if (-1 == handle) {
		return -EINVAL;
	}

	if (mCmdFd < 0) {
		ALOGE("<BST> " "cannot tx cmd: %s",
				(char *)strerror(-mCmdFd));
		return mCmdFd;
	}

	cmd.magic = CHANNEL_PKT_MAGIC_CMD;

	cmd.command.cmd = SET_SENSOR_DELAY;
	cmd.command.code = handle;
	cmd.command.value = ns / 1000000;
	cmd.ts = 0;

	s = BstSensorInfo::getSensor(id);
	ALOGI("<BST>" "set delay of <%s> to %lldms",
			(s != NULL) ? s->name : "unknown",
			ns / 1000000);

	err = write(mCmdFd, &cmd, sizeof(cmd));
	err = err < (int)sizeof(cmd) ? -1 : 0;

	if (!err) {
		mDelays[pos] = ns / 1000000;
	}

	return err;
}


int BstSensor::readEvents(sensors_event_t *pdata, int count)
{
	int rslt;
	int err;
	struct pollfd fds;
	struct timespec ts;
	int64_t time_ns;
	struct exchange sensor_data;
	int sensor;
	sensors_event_t *pdata_cur;

	if (count > 1) {
		count = 1;
	}
	rslt = 0;
	pdata_cur = pdata;
	while (rslt < count) {

#if 0
		fds.revents = 0;
		err = poll(&fds, 1, BST_DATA_POLL_TIMEOUT);
		if (err <= 0) {
			return rslt;
		}
#endif
		err = read(fds.fd, &sensor_data, sizeof(sensor_data));
		if (err < (int)sizeof(sensor_data)) {
			ALOGE("<BST> " "bad condition, stream needs sync");
			return rslt;
		}

		if (CHANNEL_PKT_MAGIC_DAT != sensor_data.magic) {
			ALOGE("<BST> " "discard invalid data packet from stream,%d,%d,%d ", sensor_data.magic, sensor_data.data.sensor, sensor_data.data.type );
			return rslt;
		}

		ts.tv_sec = ts.tv_nsec = 0;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		time_ns = ts.tv_sec * 1000000000LL + ts.tv_nsec;

		sensor = sensor_data.data.sensor;
		pdata_cur->version = sizeof(*pdata_cur);
		pdata_cur->sensor = BstSensor::handle2id(sensor);
		pdata_cur->timestamp = time_ns;

		switch (BstSensor::handle2id(sensor)) {
		case SENSOR_HANDLE_ACCELERATION:
			pdata_cur->acceleration.x = GRAVITY_EARTH *
				sensor_data.data.acceleration.x;
			pdata_cur->acceleration.y = GRAVITY_EARTH *
				sensor_data.data.acceleration.y;
			pdata_cur->acceleration.z = GRAVITY_EARTH *
				sensor_data.data.acceleration.z;
			pdata_cur->acceleration.status =
				sensor_data.data.acceleration.status;
			pdata_cur->type = SENSOR_TYPE_ACCELEROMETER;
			break;
		case SENSOR_HANDLE_MAGNETIC_FIELD:
			pdata_cur->magnetic.x = sensor_data.data.magnetic.x;
			pdata_cur->magnetic.y = sensor_data.data.magnetic.y;
			pdata_cur->magnetic.z = sensor_data.data.magnetic.z;
			pdata_cur->magnetic.status = sensor_data.data.magnetic.status;
			pdata_cur->type = SENSOR_TYPE_MAGNETIC_FIELD;
			break;
		case SENSOR_HANDLE_ORIENTATION:
			pdata_cur->orientation.azimuth = sensor_data.data.orientation.azimuth;
			pdata_cur->orientation.pitch = sensor_data.data.orientation.pitch;
			pdata_cur->orientation.roll = sensor_data.data.orientation.roll;
			pdata_cur->orientation.status = sensor_data.data.orientation.status;
			pdata_cur->type = SENSOR_TYPE_ORIENTATION;
			break;
		default:
			ALOGE("Invalid data pkt");
			return rslt;
		}


		rslt++;
		pdata_cur++;
	}

	return rslt;
}
