# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1")
  file(MAKE_DIRECTORY "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1")
endif()
file(MAKE_DIRECTORY
  "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/1"
  "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1"
  "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1/tmp"
  "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1/src/servo_v1+servo_v1-stamp"
  "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1/src"
  "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1/src/servo_v1+servo_v1-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1/src/servo_v1+servo_v1-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/penghao/Desktop/Servo_9948/G431_base_code/software/servo_v1/MDK-ARM/tmp/servo_v1/default/servo_v1+servo_v1/src/servo_v1+servo_v1-stamp${cfgdir}") # cfgdir has leading slash
endif()
