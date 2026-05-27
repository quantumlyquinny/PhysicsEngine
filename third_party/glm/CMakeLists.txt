cmake_minimum_required(VERSION 3.15)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)

# 1. Fetch SFML automatically from GitHub
include(FetchContent)
FetchContent_Declare(
    SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.1
)
FetchContent_MakeAvailable(SFML)

# 2. Automatically find all your .cpp files in the src folder
file(GLOB_RECURSE ENGINE_SOURCES "src/*.cpp")

# 3. Add the GLAD C-file
set(GLAD_SOURCE "third_party/glad/src/glad.c")

# 4. Create the main Engine executable
add_executable(PhysicsEngine ${ENGINE_SOURCES} ${GLAD_SOURCE})
set_target_properties(PhysicsEngine PROPERTIES WIN32_EXECUTABLE FALSE) # <--- ADDED THIS LINE

# 5. Tell the compiler where your GLAD and GLM folders are
target_include_directories(PhysicsEngine PRIVATE 
    "third_party/glad/include"
    "third_party" 
)

# 6. Link the SFML libraries to the engine
target_link_libraries(PhysicsEngine PRIVATE 
    sfml-graphics 
    sfml-window 
    sfml-system 
    sfml-main
)