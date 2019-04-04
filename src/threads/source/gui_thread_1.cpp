#include "gui_thread_1.h"
#include "global_struct.h"
#include <gtkmm.h>
#include <string.h>

//shared_robot_data *robot_data1;
//auto app

 Gui::Gui(void *arg) {
    this->robot_data1 = (shared_robot_data *)arg;
    Gui::init();
 }

 void Gui::init() {
     this->app = Gtk::Application::create();
     this->builder = Gtk::Builder::create_from_file("window_main.glade");
 }


void Gui::shutdown() {
    robot_data1->shutdown = true;
}

void Gui::update_values() {
    sprintf(p_value_char, "%f", p_value);
    sprintf(i_value_char, "%f", i_value);
    sprintf(d_value_char, "%f", d_value);
    sprintf(mass_value_char, "%f", mass_value);
    lbl_p_value->set_text(p_value_char);
    lbl_i_value->set_text(i_value_char);
    lbl_d_value->set_text(d_value_char);
    lbl_mass_value->set_text(mass_value_char);
    lbl_status setText(robot_data1->robot_status)
    robot_data1->kp = p_value;
    robot_data1->ki = i_value;
    robot_data1->kd = d_value;
    robot_data1->fake_mass = mass_value;
    
    //return true;
}

void Gui::change(double &value_ptr, double value) {
    value_ptr += value;
    update_values();
}

//m_button1.signal_clicked().connect( sigc::bind<Glib::ustring>( sigc::mem_fun(*this, &HelloWorld::on_button_clicked), "button 1") );

void Gui::create_button(std::string name, double &value_ptr, double change_value) {
   // this->change_value = change_value;
    Gtk::Button *btn = nullptr;
    this->builder->get_widget(name, btn);
 //   btn->signal_clicked().connect(sigc::mem_fun(*this, &Gui::change));
    btn->signal_clicked().connect([this, &value_ptr, change_value]() {
      //  *value_ptr += change_value; 
        Gui::change(value_ptr, change_value);
        });
 //   btn->signal_clicked().connect( sigc::bind<double>( sigc::mem_fun(*this, &f, value_ptr) );

}


void* Gui::GuiThread() {

   
    //auto app
    p_value = robot_data1->kp;
    i_value = robot_data1->ki;
    d_value = robot_data1->kd;
    mass_value = robot_data1->fake_mass;

    printf("running");

    builder->get_widget("window_main", mainWindow);
    builder->get_widget("btn_shutdown", btn_shutdown);
    builder->get_widget("lbl_p_value", lbl_p_value);
    builder->get_widget("lbl_i_value", lbl_i_value);
    builder->get_widget("lbl_d_value", lbl_d_value);
    builder->get_widget("lbl_mass_value", lbl_mass_value);
    builder->get_widget("lbl_status", lbl_status);


  //  create_button("btn_p_up", &p_value, 0.01);
    create_button("btn_p_up", p_value, 0.1);
    create_button("btn_p_down", p_value, -0.1);
    create_button("btn_i_up", i_value, 0.1);
    create_button("btn_i_down", i_value, -0.1);
    create_button("btn_d_up", d_value, 0.01);
    create_button("btn_d_down", d_value, -0.01);
    create_button("btn_mass_up", mass_value, 0.1);
    create_button("btn_mass_down", mass_value, -0.1);

  
    btn_shutdown->signal_clicked().connect(sigc::mem_fun(*this, &Gui::shutdown));
    
 //Glib::signal_timeout().connect(sigc::ptr_fun(&update_values), 40);

    app->run(*mainWindow);
    return NULL;
}