#include "gui_thread.h"
#include "global_struct.h"
#include "gui_thread_1.h"
//#include <gtkmm.h>



void* GuiThread(void *arg) {
    Gui gui = Gui(arg);

    gui.GuiThread();
    

    printf("running in thread");
    
    return NULL;
}