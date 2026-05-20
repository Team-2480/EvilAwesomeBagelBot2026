set(CPM_USE_LOCAL_PACKAGES OFF)
set(CPM_SOURCE_CACHE "${CMAKE_SOURCE_DIR}/.cache/cpm")
include(CPM)

function(add_maven NAME URL_PREFIX LIBTARGET LIBNAME LIBPATH HEADERS)
  CPMAddPackage(
    NAME ${NAME}-lib
    URL ${URL_PREFIX}-${LIBTARGET}.zip)

  if(${HEADERS})
    CPMAddPackage(
      NAME ${NAME}-headers
      URL ${URL_PREFIX}-headers.zip)
  endif()

  add_library(${NAME} INTERFACE)
  if (${HEADERS})
    target_include_directories(${NAME} INTERFACE ${${NAME}-headers_SOURCE_DIR})
  endif()

  target_link_libraries(${NAME} INTERFACE ${${NAME}-lib_SOURCE_DIR}${LIBPATH}/${LIBNAME})
endfunction()

add_maven(REVLib-cpp https://maven.revrobotics.com/com/revrobotics/frc/REVLib-cpp/2026.0.5/REVLib-cpp-2026.0.5 linuxx86-64 libREVLib.so /linux/x86-64/shared ON)
add_maven(REVLib-driver https://maven.revrobotics.com/com/revrobotics/frc/REVLib-driver/2026.0.5/REVLib-driver-2026.0.5 linuxx86-64 libREVLibDriver.so /linux/x86-64/shared ON)
add_maven(RevLibBackendDriver https://maven.revrobotics.com/com/revrobotics/frc/RevLibBackendDriver/2026.0.1/RevLibBackendDriver-2026.0.1 linuxx86-64 libBackendDriver.so /linux/x86-64/shared OFF)
add_maven(wpiapi-cpp https://maven.ctr-electronics.com/release/com/ctre/phoenix6/wpiapi-cpp/26.1.0/wpiapi-cpp-26.1.0 linuxx86-64 libCTRE_Phoenix6_WPI.so /linux/x86-64/shared ON)
add_maven(ctretools https://maven.ctr-electronics.com/release/com/ctre/phoenix6/tools/26.1.0/tools-26.1.0 linuxx86-64 libCTRE_PhoenixTools.so /linux/x86-64/shared ON)
