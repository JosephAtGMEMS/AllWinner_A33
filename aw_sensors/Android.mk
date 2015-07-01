# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
LOCAL_PATH := $(call my-dir)
# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)
LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
#LOCAL_MODULE := sensors.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE :=sensors.exdroid
LOCAL_MODULE_TAGS := eng optional

ifeq ($(SW_BOARD_USES_MAGSENSOR_TYPE), fxos8700 )
LOCAL_CPPFLAGS += -DMAG_SENSOR_FXOS8700
LOCAL_SRC_FILES := 						\
				sensors.cpp 			\
				sensorDetect.cpp                \
				insmodDevice.cpp                \
				SensorBase.cpp			\
				AccelSensor.cpp			\
				MagSensor.cpp                   \
				GyroSensor.cpp                  \
				LightSensor.cpp                 \
				ProximitySensor.cpp             \
				TempSensor.cpp                  \
				PressSensor.cpp			\
                                InputEventReader.cpp            
    
else
LOCAL_SRC_FILES := 						\
				sensors.cpp 			\
				sensorDetect.cpp                \
				insmodDevice.cpp                \
				SensorBase.cpp			\
				AccelSensor.cpp			\
				LightSensor.cpp                 \
                                ProximitySensor.cpp             \
                                TempSensor.cpp                  \
				PressSensor.cpp			\
                                InputEventReader.cpp 
   
endif      

#				MagnetoSensor.cpp               \
#				GyroSensor.cpp                  \

LOCAL_SHARED_LIBRARIES := liblog libcutils libdl

LOCAL_LDFLAGS = $(LOCAL_PATH)/LibFusion_ARM_cpp.a

include $(BUILD_SHARED_LIBRARY)
