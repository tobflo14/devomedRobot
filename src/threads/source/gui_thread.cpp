#include "gui_thread.h"
#include "global_struct.h"
#include "graphics/headers/gui.h"
//#include <gtkmm.h>



void* GuiThread(void *arg) {
    Gui gui = Gui(arg);

    gui.GuiThread();
    
    printf("running in thread");
    
    return NULL;
}