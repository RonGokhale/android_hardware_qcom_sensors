/*
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
#include <string.h>
#include <sys/select.h>
#include "LightSensor.h"
#include <cutils/log.h>

#define SENSOR_NAME     "lightsensor-level"
/*****************************************************************************/
LightSensor::LightSensor(): SensorBase(NULL, SENSOR_NAME),
	mInputReader(32),
	alsEnabled(0),
	psEnabled(0),
	mHasPendingEvent(false)
{
	memset(&mPendingEvent, 0, sizeof(mPendingEvent));
	if(sensor_get_class_path()<0)
		ALOGD("light sensor get class path error \n");
	else
		ALOGD("LightSensor::LightSensor sensor path is %s\n",class_path);
}

LightSensor::~LightSensor(){
}

int LightSensor::setInitialState() {
	return 0;
}

int LightSensor::enable(int32_t handle, int en) {
	char buffer[20];
	int als = 0;
	int ps = 0;
	int count = 0;
	if (handle == ID_L)  {
		if (!en)
		{
			alsEnabled = 0;
		}else
		{
			alsEnabled = 1;
			als = 1;
		}
		ALOGD(" %s Light Sensor En=%d\n", __func__, en);
		count = sprintf(buffer, "%d\n", als);
		set_sysfs_input_attr(class_path,"enable",buffer,count);
	}
	return 0;
}

bool LightSensor::hasPendingEvents() const {
	return mHasPendingEvent;
}

int LightSensor::readEvents(sensors_event_t* data, int count)
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
		if (type == EV_ABS) {
			processEvent(event->code, event->value);
		} else if (type == EV_SYN) {
			int64_t time = timevalToNano(event->time);

			mPendingEvent.timestamp = time;
			*data++ = mPendingEvent;
			count--;
			numEventReceived++;

		} else {
			ALOGE(" Light Sensor: unknown event (type=%d, code=%d)",
					type, event->code);
		}
		mInputReader.next();
	}
	return numEventReceived;
}

void LightSensor::processEvent(int code, int value)
{
	switch (code) {
		case ABS_MISC:
			mPendingEvent.version = sizeof(sensors_event_t);
			mPendingEvent.sensor = ID_L;
			mPendingEvent.type = SENSOR_TYPE_LIGHT;
			memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
			mPendingEvent.light = value;
			ALOGD("Light mPendingEvent.light = %d\n", value);
			break;
		default:
			break;
	}

}
int LightSensor::getFd() const
{
	return data_fd;
}

int LightSensor::sensor_get_class_path()
{
	char dirname[] = "/sys/class/input";
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
		if (strcmp(buf, SENSOR_NAME) == 0) {
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

int LightSensor:: set_sysfs_input_attr(char *class_path,
		const char *attr, char *value, int len)
{
	char path[256];
	int fd;

	if (class_path == NULL || *class_path == '\0'
			|| attr == NULL || value == NULL || len < 1) {
		return -EINVAL;
	}
	snprintf(path, sizeof(path), "%s/%s", class_path, attr);
	path[sizeof(path) - 1] = '\0';
	fd = open(path, O_RDWR);
	if (fd < 0) {
		return -errno;
	}
	if (write(fd, value, len) < 0) {
		close(fd);
		return -errno;
	}
	close(fd);
	return 0;
}
