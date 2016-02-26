/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2008 The Android Open Source Project
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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <cutils/log.h>

#include "BstSensorAccel.h"

#define DEBUG_SENSOR    0

struct BmaAxisRemap{
     int DstAxis;
     int Sign;
};

BstSensorAccel::BstSensorAccel(const char* InputName, BmaLayout Layout)
: SensorBase(NULL, InputName),
      mEnabled(0),
      mPendingMask(0),
      mInputReader(32),
      mDelay(0),
      mLayout(Layout)
{
    memset(&mPendingEvent, 0, sizeof(mPendingEvent));
	memset(mClassPath, '\0', sizeof(mClassPath));

    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor  = ID_A;
    mPendingEvent.type    = SENSOR_TYPE_ACCELEROMETER;
    mPendingEvent.acceleration.status = SENSOR_STATUS_ACCURACY_HIGH;

	if(sensor_get_class_path(mClassPath, InputName))
	{
		ALOGE("Can`t find the %s sensor!", InputName);
	}

#if DEBUG_SENSOR
    ALOGE("<BST> sensor: mLaout = %d\n", mLayout);
#endif
    if(mLayout >= BMA_LAYOUT_MAX)
    {
        mLayout = TOP_Y_FORWARD;
    }
}

BstSensorAccel::~BstSensorAccel()
{
}

int BstSensorAccel::enable(int32_t handle, int en)
{
	int err = 0;
    uint32_t newState  = en;

	ALOGD("<BST> %s BMA250, orig state: %d",
			en ? "enable" : "disable", mEnabled);

    if (mEnabled != newState) {
        if (newState && !mEnabled)
            err = enable_sensor();
        else if (!newState)
            err = disable_sensor();
		ALOGI("<BST> Change BMA250 sensor state, enabled: %d, newstate %d", mEnabled, newState);
        ALOGE_IF(err, "<BST> Could not change BMA250 sensor state, enabled: %d, newstate %d (%s).", mEnabled, newState, strerror(-err));
        if (!err) {
            mEnabled = newState;
            update_delay();
        }
    }
    return err;
}

int BstSensorAccel::setDelay(int32_t handle, int64_t ns)
{
    if (ns < 0)
        return -EINVAL;

    mDelay = ns;
    return update_delay();
}

int BstSensorAccel::update_delay()
{
    if (mEnabled) {
        return set_delay(mDelay);
    }
    else
	    return 0;
}

int BstSensorAccel::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if ((type == EV_ABS) || (type == EV_REL) || (type == EV_KEY)) {
            processEvent(event->code, event->value);
            mInputReader.next();
        } else if (type == EV_SYN) {
            int64_t time = timevalToNano(event->time);
#if DEBUG_SENSOR
			ALOGE("<BST> Sensor event time: %f\n",
					(int64_t)((int64_t)event->time.tv_sec*1000000000
					+ (int64_t)event->time.tv_usec*1000) / 1000000000.0);
#endif
			if (mPendingMask) {
				mPendingMask = 0;
				mPendingEvent.timestamp = time;
				if (mEnabled) {
					*data++ = mPendingEvent;
					count--;
					numEventReceived++;
				}
			}
            if (!mPendingMask) {
                mInputReader.next();
            }
        } else {
            ALOGE("BstSensorAccel: unknown event (type=%d, code=%d)",
                    type, event->code);
            mInputReader.next();
        }
    }

    return numEventReceived;
}

void BstSensorAccel::processEvent(int code, int value)
{

    switch (code) {
        case ABS_X:
            mPendingMask = 1;
            RemapAxis(0, value, &(mPendingEvent.acceleration));
            break;
        case ABS_Y:
            mPendingMask = 1;
            RemapAxis(1, value, &(mPendingEvent.acceleration));
            break;
        case ABS_Z:
            mPendingMask = 1;
            RemapAxis(2, value, &(mPendingEvent.acceleration));
            break;
		default:
			/* TODO: implement if needed. */
			break;
    }

#if DEBUG_SENSOR
	ALOGE("the bma250 code is %d value is %d\n" ,code, value);
	ALOGE("the bma250 data is x= %f , y = %f , z = %f\n",mPendingEvent.acceleration.x,\
							mPendingEvent.acceleration.y,mPendingEvent.acceleration.z);
#endif
}

int BstSensorAccel::writeDisable(int isDisable) {
	char attr[PATH_MAX] = {'\0'};
	if(mClassPath[0] == '\0')
		return -1;

	strcpy(attr, mClassPath);
	strcat(attr,"/");
	strcat(attr,"enable");

	int fd = open(attr, O_RDWR);
	if (0 > fd) {
		ALOGE("Could not open (write-only) SysFs attribute \"%s\" (%s).", attr, strerror(errno));
		return -errno;
	}

	char buf[2];

	if (isDisable) {
		buf[0] = '0';
	} else {
		buf[0] = '1';
	}
	buf[1] = '\0';

	int err = 0;
	err = write(fd, buf, sizeof(buf));

	if (0 > err) {
		err = -errno;
		ALOGE("Could not write SysFs attribute \"%s\" (%s).", attr, strerror(errno));
	} else {
		err = 0;
	}

	close(fd);

	return err;
}

int BstSensorAccel::writeDelay(int64_t ns) {
	char attr[PATH_MAX] = {'\0'};
	if(mClassPath[0] == '\0')
		return -1;

	strcpy(attr, mClassPath);
	strcat(attr,"/");
	strcat(attr,"delay");

	int fd = open(attr, O_RDWR);
	if (0 > fd) {
		ALOGE("Could not open (write-only) SysFs attribute \"%s\" (%s).", attr, strerror(errno));
		return -errno;
	}
	if (ns > 10240000000LL) {
		ns = 10240000000LL; /* maximum delay in nano second. */
	}
	if (ns < 312500LL) {
		ns = 312500LL; /* minimum delay in nano second. */
	}

    char buf[80];
    sprintf(buf, "%lld", ns/1000/1000);
    write(fd, buf, strlen(buf)+1);
    close(fd);
    return 0;

}

void BstSensorAccel::RemapAxis(int SrcAxis, int SrcValue, sensors_vec_t* Vector) {
    if(SrcAxis > 2)
    {
        /*0: x-axis; 1: y-axis; 2: z-axis; other: invalid*/
        return;
    }

    /*convert unit*/
    float Value = SrcValue*(4*GRAVITY_EARTH)/(float)(1 << 10);
    /**************rule for remapping axis***********************
    * x',y',z':     destination axis (Android coordinate-system)
    * x ,y ,z :     source axis(Sensor h/w coordinate-system)
    *------------------------------------------------------------
    * ************the definition is from top view****************
    * TOP_Y_FORWARD           :   x=> x'; y=> y'; z=> z';
    * TOP_Y_RIGHTWARD         :  -x=> y'; y=> x'; z=> z';
    * TOP_Y_BACKWARD          :  -x=> x';-y=> y'; z=> z';
    * TOP_Y_LEFTWARD          :   x=> y';-y=> x'; z=> z';
    * BOTTOM_Y_FORWAR         :  -x=> x'; y=> y';-z=> z';
    * BOTTOM_Y_RIGHTWARD      :   x=> y'; y=> x';-z=> z';
    * BOTTOM_Y_BACKWARD       :   x=> x':-y=> y';-z=> z';
    * BOTTOM_Y_LEFTWARD       :  -x=> y';-y=> x';-z=> z';
    *************************************************************/
    static const BmaAxisRemap RemapTable[BMA_LAYOUT_MAX][3]=
    {
        {{0, 1},{1, 1},{2, 1}},/*TOP_Y_FORWARD*/
        {{1,-1},{0, 1},{2, 1}},/*TOP_Y_RIGHTWARD*/
        {{0,-1},{1,-1},{2, 1}},/*TOP_Y_BACKWARD*/
        {{1, 1},{0,-1},{2, 1}},/*TOP_Y_LEFTWARD*/
        {{0,-1},{1, 1},{2,-1}},/*BOTTOM_Y_FORWAR*/
        {{1, 1},{0, 1},{2,-1}},/*BOTTOM_Y_RIGHTWARD*/
        {{0, 1},{1,-1},{2,-1}},/*BOTTOM_Y_BACKWARD*/
        {{1,-1},{0,-1},{2,-1}},/*BOTTOM_Y_LEFTWARD*/
    };
    Vector->v[RemapTable[mLayout][SrcAxis].DstAxis] = Value*RemapTable[mLayout][SrcAxis].Sign;
}

int BstSensorAccel::enable_sensor() {
	return writeDisable(0);
}

int BstSensorAccel::disable_sensor() {
	return writeDisable(1);
}

int BstSensorAccel::set_delay(int64_t ns) {
	return writeDelay(ns);
}

int BstSensorAccel::sensor_get_class_path(char *class_path, const char* InputName)
{
	const char* dirname = "/sys/class/input";
	char buf[256];
	int res;
	DIR *dir;
	struct dirent *de;
	int fd = -1;
	int found = 0;

	dir = opendir(dirname);
	if (dir == NULL)
		return -1;

	while((de = readdir(dir))) {
		if (strncmp(de->d_name, "input", strlen("input")) != 0) {
		    continue;
		}

		sprintf(class_path, "%s/%s", dirname, de->d_name);
		snprintf(buf, sizeof(buf), "%s/name", class_path);

		fd = open(buf, O_RDONLY);
		if (fd < 0) {
		    continue;
		}
		if ((res = read(fd, buf, sizeof(buf))) < 0) {
		    close(fd);
		    continue;
		}
		buf[res - 1] = '\0';
		if (strcmp(buf, InputName) == 0) {
		    found = 1;
		    close(fd);
		    break;
		}

		close(fd);
		fd = -1;
	}
	closedir(dir);

	if (found) {
		return 0;
	}else {
		*class_path = '\0';
		return -1;
	}
}
