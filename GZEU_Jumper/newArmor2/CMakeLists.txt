cmake_minimum_required(VERSION 3.5.1)
PROJECT(main)
find_package(OpenCV 4.5.2)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(main 
/home/jumper/GZEU_Jumper/newArmor2/Src/main.cpp 
/home/jumper/GZEU_Jumper/newArmor2/Src/armorDetector.cpp
/home/jumper/GZEU_Jumper/newArmor2/Src/mathTools.cpp
/home/jumper/GZEU_Jumper/newArmor2/include/mathTools.h
/home/jumper/GZEU_Jumper/newArmor2/Src/armorBox.cpp
/home/jumper/GZEU_Jumper/newArmor2/include/Armor.h
/home/jumper/GZEU_Jumper/newArmor2/include/CameraApi.h
/home/jumper/GZEU_Jumper/newArmor2/include/serial.hpp
/home/jumper/GZEU_Jumper/newArmor2/Src/serial.cpp
/home/jumper/GZEU_Jumper/newArmor2/Src/ThreadControl.cpp
/home/jumper/GZEU_Jumper/newArmor2/include/ThreadControl.h

)
target_link_libraries(main ${OpenCV_LIBS})
find_package(Threads)
target_link_libraries(main ${CMAKE_THREAD_LIBS_INIT})
find_package(X11 REQUIRED)
include_directories(${X11_INLUCDE_DIR})
target_link_libraries(main ${X11_LIBRARIES})
target_link_libraries(main /home/jumper/GZEU_Jumper/newArmor2/mindvision/linux/lib/x64/libMVSDK.so)
#target_link_libraries(main /home/jumper/GS/newArmor2/camera/libdvp.so)
#target_link_libraries(main /home/jumper/GS/newArmor2/camera/libhzd.so)
#target_link_libraries(main /home/jumper/GS/newArmor2/camera/usb3_m3s_all.dscam.so)
#target_link_libraries(main /home/gs/GS/newArmor2/camera/libD3tKit.so)

