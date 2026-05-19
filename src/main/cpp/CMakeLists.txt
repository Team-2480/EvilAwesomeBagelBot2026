add_executable(robotmain
  Main.cpp
  subsystems/Intake.cpp
  subsystems/Agitate.cpp
  subsystems/Climb.cpp
  subsystems/Shooter.cpp
  subsystems/DriveSubsystem.cpp
  subsystems/MAXSwerveModule.cpp
  Robot.cpp
  pid.cpp
)

add_compile_options(-Wno-array-bounds)

set(FETCHCONTENT_QUIET FALSE)
set(WITH_GUI OFF)
set(WITH_TESTS OFF)
set(WITH_WPILIB ON)
include(FetchContent)
FetchContent_Declare(wpilib
  URL https://github.com/wpilibsuite/allwpilib/archive/refs/tags/v2026.2.1.tar.gz
)
FetchContent_Declare(pathplanner
  GIT_REPOSITORY https://github.com/Team-2480/pathplanner-cmake
  GIT_PROGRESS On
  GIT_TAG cmake
)
FetchContent_MakeAvailable(wpilib pathplanner)

target_link_libraries(robotmain wpilibc wpilibNewCommands REVLib-cpp wpiapi-cpp pathplanner ctretools REVLib-driver RevLibBackendDriver)
target_include_directories(robotmain PUBLIC ./)

add_custom_command(TARGET robotmain POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Add clangd to the base directory")

install(TARGETS robotmain DESTINATION bin)
