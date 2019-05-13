#include "gui_thread.h"
#include "global_struct.h"
#include "graphics/headers/gui.h"
//#include <gtkmm.h>



void* GuiThread(void *arg) {
    std::cout << "Running in GUI-thread" << std::endl;
    
    Gui gui = Gui(arg);
    std::cout << "inbetween" << std::endl;
    gui.GuiThread();
    
    std::cout << "Exit GUI-thread" << std::endl;
    
    return NULL;
}