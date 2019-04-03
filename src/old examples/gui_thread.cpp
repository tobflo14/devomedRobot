#include "gui_thread.h"
#include "global_struct.h"
#include "gui_thread_1.h"
#include <gtkmm.h>

 
Gtk::Button *btn_p_up = nullptr;
Gtk::Button *btn_p_down = nullptr;
Gtk::Button *btn_i_up = nullptr;
Gtk::Button *btn_i_down = nullptr;
Gtk::Button *btn_d_up = nullptr;
Gtk::Button *btn_d_down = nullptr;
Gtk::Button *btn_shutdown = nullptr;
Gtk::Label  *lbl_p_value = nullptr;
Gtk::Label  *lbl_i_value = nullptr;
Gtk::Label  *lbl_d_value = nullptr;
shared_robot_data *robot_data;

double p_value;
double i_value;
double d_value;

char p_value_char[30];
char i_value_char[30] = {0};
char d_value_char[30] = {0};

static unsigned int counter = 0;
char str_count[30] = {0};

Gtk::Window *mainWindow = nullptr;
auto app = Gtk::Application::create();

void shutdown() {
 
    robot_data->shutdown = true;
   
  //  mainWindow->hide();
    
  // 
}

void update_values() {
    sprintf(p_value_char, "%f", p_value);
    sprintf(i_value_char, "%f", i_value);
    sprintf(d_value_char, "%f", d_value);
    lbl_p_value->set_text(p_value_char);
    lbl_i_value->set_text(i_value_char);
    lbl_d_value->set_text(d_value_char);
    robot_data->kp = p_value;
    robot_data->ki = i_value;
    robot_data->kd = d_value;
    //return true;
}


void p_up() {
    p_value += 0.1;
    update_values();
}
void p_down() {
    p_value -= 0.1;
    update_values();
}
void i_up() {
    i_value += 0.1;
    update_values();
}
void i_down() {
    i_value -= 0.1;
    update_values();
}
void d_up() {
    d_value += 0.001;
    update_values();
}
void d_down() {
    d_value -= 0.001;
    update_values();
}


void* GuiThread(void *arg) {

    

    robot_data = (shared_robot_data *)arg;
    Gui gui = Gui(robot_data);

    gui.GuiThread();
    
    p_value = robot_data->kp;
    i_value = robot_data->ki;
    d_value = robot_data->kd;

    printf("running");
    
    
    
    Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("window_main.glade");



    builder->get_widget("window_main", mainWindow);
    builder->get_widget("btn_p_up", btn_p_up);
    builder->get_widget("btn_p_down", btn_p_down);
    builder->get_widget("btn_i_up", btn_i_up);
    builder->get_widget("btn_i_down", btn_i_down);
    builder->get_widget("btn_d_up", btn_d_up);
    builder->get_widget("btn_d_down", btn_d_down);
    builder->get_widget("btn_shutdown", btn_shutdown);
    builder->get_widget("lbl_p_value", lbl_p_value);
    builder->get_widget("lbl_i_value", lbl_i_value);
    builder->get_widget("lbl_d_value", lbl_d_value);

    btn_p_up->signal_clicked().connect(sigc::ptr_fun(&p_up));
    btn_p_down->signal_clicked().connect(sigc::ptr_fun(&p_down));
    btn_i_up->signal_clicked().connect(sigc::ptr_fun(&i_up));
    btn_i_down->signal_clicked().connect(sigc::ptr_fun(&i_down));
    btn_d_up->signal_clicked().connect(sigc::ptr_fun(&d_up));
    btn_d_down->signal_clicked().connect(sigc::ptr_fun(&d_down));

    btn_shutdown->signal_clicked().connect(sigc::ptr_fun(&shutdown));
    //Glib::signal_timeout().connect(sigc::ptr_fun(&update_values), 40);

    app->run(*mainWindow);
    
    return NULL;
}