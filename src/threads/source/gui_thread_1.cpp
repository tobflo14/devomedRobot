#include "gui_thread_1.h"
#include "global_struct.h"
#include <gtkmm.h>
#include <string.h>
#include <gtkmm/glarea.h>
#include <plot2d.h>

//shared_robot_data *robot_data1;
//auto app

Plot2d plot = Plot2d(100,5,2);

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
    //lbl_status setText(robot_data1->robot_status)
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
    builder->get_widget("gl_area", gl_area);


  //  create_button("btn_p_up", &p_value, 0.01);
    create_button("btn_p_up", p_value, 0.1);
    create_button("btn_p_down", p_value, -0.1);
    create_button("btn_i_up", i_value, 1);
    create_button("btn_i_down", i_value, -1);
    create_button("btn_d_up", d_value, 10);
    create_button("btn_d_down", d_value, -10);
    create_button("btn_mass_up", mass_value, 0.1);
    create_button("btn_mass_down", mass_value, -0.1);

  
    btn_shutdown->signal_clicked().connect(sigc::mem_fun(*this, &Gui::shutdown));

    //plot.gl_draw();
 //Glib::signal_timeout().connect(sigc::ptr_fun(&update_values), 40);
    gl_area->set_auto_render();
    gl_area->signal_realize().connect(sigc::mem_fun(*this, &Gui::onRealize));
    gl_area->signal_render().connect(sigc::mem_fun(*this, &Gui::onRender), false);
    /* gl_area->signal_render().connect([this]() {
      //  *value_ptr += change_value; 
        Gui::onRender();
        }); */

    app->run(*mainWindow);
    return NULL;
}


void Gui::onRealize(){
    gl_area->make_current();
    try {
        plot.realize();
    }
    catch(const Gdk::GLError& gle) {
        std::cerr << "An error occured during realize:" << std::endl;
        std::cerr << gle.domain() << "-" << gle.code() << "-" << gle.what() << std::endl;
    }

}

void Gui::onUnrealize(){
    //clean up 
}

bool Gui::onRender(const Glib::RefPtr<Gdk::GLContext>& /* context */) {
    
     try
  {
    gl_area->throw_if_error();
    plot.gl_draw();
  }
  catch(const Gdk::GLError& gle)
  {
    cerr << "An error occured making the context current during realize:" << endl;
    cerr << gle.domain() << "-" << gle.code() << "-" << gle.what() << endl;
  }
    
    return true;
}
/*

void Gui::draw_triangle()
{
  if (this->program == 0 || this->vao == 0)
    return;

  glUseProgram (this->program);

  glUniformMatrix4fv (self->mvp_location, 1, GL_FALSE, &(self->mvp[0]));

  glBindVertexArray (self->vao);

  glDrawArrays (GL_TRIANGLES, 0, 3);

  glBindVertexArray (0);
  glUseProgram (0);
}

static gboolean
gl_draw ()
{
  glClearColor (0.5, 0.5, 0.5, 1.0);
  glClear (GL_COLOR_BUFFER_BIT);

  draw_triangle (self);

  glFlush ();

  return FALSE;
}
*/