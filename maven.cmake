include(ExternalProject)
function(add_maven NAME URL_PREFIX LIBTARGET LIBNAME LIBPATH HEADERS)
  ExternalProject_Add(
    ${NAME}-lib
    PREFIX ${NAME}
    URL ${URL_PREFIX}-${LIBTARGET}.zip
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
  )
  if(${HEADERS})
    ExternalProject_Add(
      ${NAME}-headers
      PREFIX ${NAME}
      URL ${URL_PREFIX}-headers.zip
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
    )
  endif()

  add_library(${NAME} INTERFACE)
  if(${HEADERS})
    ExternalProject_Get_Property(${NAME}-headers SOURCE_DIR)
    target_include_directories(${NAME} INTERFACE ${SOURCE_DIR})
  endif()
  ExternalProject_Get_Property(${NAME}-lib SOURCE_DIR)
  target_link_libraries(${NAME} INTERFACE ${SOURCE_DIR}${LIBPATH}/${LIBNAME})
endfunction()

add_maven(REVLib-cpp https://maven.revrobotics.com/com/revrobotics/frc/REVLib-cpp/2026.0.5/REVLib-cpp-2026.0.5 linuxx86-64 libREVLib.so /linux/x86-64/shared ON)
add_maven(REVLib-driver https://maven.revrobotics.com/com/revrobotics/frc/REVLib-driver/2026.0.5/REVLib-driver-2026.0.5 linuxx86-64 libREVLibDriver.so /linux/x86-64/shared ON)
add_maven(RevLibBackendDriver https://maven.revrobotics.com/com/revrobotics/frc/RevLibBackendDriver/2026.0.1/RevLibBackendDriver-2026.0.1 linuxx86-64 libBackendDriver.so /linux/x86-64/shared OFF)
add_maven(wpiapi-cpp https://maven.ctr-electronics.com/release/com/ctre/phoenix6/wpiapi-cpp/26.1.0/wpiapi-cpp-26.1.0 linuxx86-64 libCTRE_Phoenix6_WPI.so /linux/x86-64/shared ON)
add_maven(ctretools https://maven.ctr-electronics.com/release/com/ctre/phoenix6/tools/26.1.0/tools-26.1.0 linuxx86-64 libCTRE_PhoenixTools.so /linux/x86-64/shared ON)
