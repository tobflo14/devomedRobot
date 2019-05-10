#include "gui_thread.h"
#include "global_struct.h"
#include "graphics/headers/gui.h"
//#include <gtkmm.h>



void* GuiThread(void *arg) {
    //Plot3d plot3 = Plot3d(); 
    Gui gui = Gui(arg);
    printf("possible to make an instance of gui");
    gui.GuiThread();
    
    printf("running in thread");
    
    return NULL;
}