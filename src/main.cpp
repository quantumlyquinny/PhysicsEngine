#include "core/Engine.hpp"

// The standard C++ main function, with arguments so MinGW doesn't panic
int main(int argc, char* argv[]) {
    
    // 1. Create the engine (Window Width, Window Height, Window Title)
    Engine engine(1280, 720, "My Custom Physics Engine");
    
    // 2. Start the main game loop!
    engine.run();

    return 0;
}