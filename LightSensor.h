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

#ifndef ANDROID_LIGHT_SENSOR_H
#define ANDROID_LIGHT_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"

/*****************************************************************************/

struct input_event;

class LightSensor : public SensorBase {
public:
            LightSensor();
    virtual ~LightSensor();

    virtual int enable(int32_t handle, int enabled);
    virtual int readEvents(sensors_event_t* data, int count);
    virtual bool hasPendingEvents(void) const;
    void processEvent(int code, int value);
    int setInitialState(void);
    int getFd() const;
    InputEventCircularReader mInputReader;
    int alsEnabled;
    int psEnabled;
    bool mHasPendingEvent;
private:
	int sensor_get_class_path();
	int set_sysfs_input_attr(char *class_path,
					const char *attr, char *value, int len);
    sensors_event_t mPendingEvent;
	char class_path[256];
};


/*****************************************************************************/

#endif  // ANDROID_LIGHT_SENSOR_H
