#include "gui.h"
#include "global_struct.h"
#include <gtkmm.h>
#include <string.h>
#include <gtkmm/glarea.h>
#include <plot2d.h>
//#include "plot3d.h"

//shared_robot_data *robot_data;
//auto app

//
Plot2d plot = Plot2d(1000,2,2); 
Plot3d plot3 = Plot3d();


 Gui::Gui(void *arg) {
    this->robot_data = (shared_robot_data *)arg;
    Gui::init();
 }

 void Gui::init() {
    
     
    // this->plot3 = Plot3d();
 }


void Gui::shutdown() {
    robot_data->shutdown = true;
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
    robot_data->kp = p_value;
    robot_data->ki = i_value;
    robot_data->kd = d_value;
    robot_data->fake_mass = mass_value;
    
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
  Glib::RefPtr<Gtk::Application> app = Gtk::Application::create();
  // = Gtk::Builder::create_from_file("window_main.glade");
    // this->app = Gtk::Application::create();
     this->builder = Gtk::Builder::create_from_file("window_main.glade");


    //auto app
    p_value = robot_data->kp;
    i_value = robot_data->ki;
    d_value = robot_data->kd;
    mass_value = robot_data->fake_mass;

    std::cout << "Running in gui main function" << std::endl;

    builder->get_widget("window_main", mainWindow);
    builder->get_widget("btn_shutdown", btn_shutdown);
    builder->get_widget("btn_record", btn_record);
    builder->get_widget("lbl_p_value", lbl_p_value);
    builder->get_widget("lbl_i_value", lbl_i_value);
    builder->get_widget("lbl_d_value", lbl_d_value);
    builder->get_widget("lbl_mass_value", lbl_mass_value);
    builder->get_widget("gl_area", gl_area);
    builder->get_widget("gl_model_area", gl_model_area);

    std::cout << "gets all widgets" << std::endl;

  //  create_button("btn_p_up", &p_value, 0.01);
    create_button("btn_p_up", p_value, 1);
    create_button("btn_p_down", p_value, -1);
    create_button("btn_i_up", i_value, 0.01);
    create_button("btn_i_down", i_value, -0.01);
    create_button("btn_d_up", d_value, 0.001);
    create_button("btn_d_down", d_value, -0.001);
    create_button("btn_mass_up", mass_value, 1);
    create_button("btn_mass_down", mass_value, -1);
    std::cout << "creates alle buttons" << std::endl;

  
    btn_shutdown->signal_clicked().connect(sigc::mem_fun(*this, &Gui::shutdown));
    btn_record->signal_clicked().connect([&]() {
      robot_data->track_position = !robot_data->track_position;
    });

    // GL PLOT AREA   
      std::cout << "more buttons" << std::endl;

 
   // gl_area->set_auto_render();
    
    gl_area->signal_realize().connect(sigc::mem_fun(*this, &Gui::onRealize));
    gl_area->signal_unrealize().connect(sigc::mem_fun(*this, &Gui::onUnrealize));
    gl_area->signal_render().connect(sigc::mem_fun(*this, &Gui::onRender), false);
    //Glib::signal_timeout().connect(sigc::mem_fun(*this, &Gui::update_plot), 4000); // gir seg_fault men virker overflødig ? 


  //  gl_model_area->set_auto_render();
    gl_model_area->signal_realize().connect(sigc::mem_fun(*this, &Gui::onRealizeModel));
    gl_model_area->signal_unrealize().connect(sigc::mem_fun(*this, &Gui::onUnrealizeModel));
    gl_model_area->signal_render().connect(sigc::mem_fun(*this, &Gui::onRenderModel), false);
    //Glib::signal_timeout().connect(sigc::mem_fun(*this, &Gui::update_model), 4000);

    mainWindow->signal_key_press_event().connect(sigc::mem_fun(*this, &Gui::onKeyPress), false);
    // Mulig problem her, hvis man trykker på knappene så endrer figuren selv om man ikke er i riktig vindu
/*
    */
    //Glib::signal_timeout().connect([this]() {
    //  Gui::update_plot();}, 1000);
    /* gl_area->signal_render().connect([this]() {
      //  *value_ptr += change_value; 
        Gui::onRender();
        }); */



    // GL MODEL AREA
   
    std::cout << "right before app run" << std::endl;
    app->run(*mainWindow);
    std::cout << "right after app run" << std::endl;
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
  gl_area->make_current();
  try {
    gl_area->throw_if_error();
    plot.unrealize();
  }
  catch(const Gdk::GLError& gle)
  {
  cerr << "An error occured making the context current during unrealize" << endl;
  cerr << gle.domain() << "-" << gle.code() << "-" << gle.what() << endl;
  }
}

bool Gui::onRender(const Glib::RefPtr<Gdk::GLContext>& /* context */) {
  gl_area->make_current();  
  try {
    gl_area->throw_if_error();
    update_plot();
    plot.gl_draw();  
  }
  catch(const Gdk::GLError& gle) {
    cerr << "An error occured making the context current during realize:" << endl;
    cerr << gle.domain() << "-" << gle.code() << "-" << gle.what() << endl;
  }
    
  return true;
}


void Gui::onRealizeModel(){
 // gl_model_area->make_current();
  gl_model_area->make_current();
  try {
  // plot3.loadOBJ();
    plot3.realize();
    std::cout << "model realized" << std::endl;
  }
  catch(const Gdk::GLError& gle) {
      std::cerr << "An error occured during realize:" << std::endl;
      std::cerr << gle.domain() << "-" << gle.code() << "-" << gle.what() << std::endl;
  }
}

void Gui::onUnrealizeModel(){
  gl_model_area->make_current();
  try {
    gl_model_area->throw_if_error();
    plot3.cleanupModel();
    std::cout << "model unrealized" << std::endl;
  }
  catch(const Gdk::GLError& gle)
  {
  cerr << "An error occured making the context current during unrealize" << endl;
  cerr << gle.domain() << "-" << gle.code() << "-" << gle.what() << endl;
  }
}

bool Gui::onRenderModel(const Glib::RefPtr<Gdk::GLContext>& /* context */) {
 // cout << "on render model" << endl;
  gl_model_area->make_current();

  try{
    gl_model_area->throw_if_error();
    plot3.drawModel(); 
  }
  catch(const Gdk::GLError& gle){
    cerr << "An error occured making the context current during realize:" << endl;
    cerr << gle.domain() << "-" << gle.code() << "-" << gle.what() << endl;
  } 
  return true;
}



bool Gui::update_plot() {
  double value1 = (double) rand() / (double) RAND_MAX;
  double value2 = rand() / RAND_MAX;
  robot_data->plot1.push_back(Point(value1, value1));
  robot_data->plot2.push_back(Point(value2, 0));

  //for (size_t i = 0; i < robot_data->plot1.size(); i++) {
  if (robot_data->plot1.size() > 0) {
      Point values[] = {robot_data->plot1[0], robot_data->plot2[0]};
      plot.graph_update(values);
  }
 // }
  robot_data->plot1.clear();
  robot_data->plot2.clear(); 
  gl_area->queue_render();
  return true;
}

bool Gui::update_model() {
  /*
  double value1 = (double) rand() / (double) RAND_MAX;
  double value2 = rand() / RAND_MAX;
  robot_data->plot1.push_back(Point(value1, value1));
  robot_data->plot2.push_back(Point(value2, 0));

  //for (size_t i = 0; i < robot_data->plot1.size(); i++) {
  if (robot_data->plot1.size() > 0) {
      Point values[] = {robot_data->plot1[0], robot_data->plot2[0]};
      //plot.graph_update(values);
  }
 // }
  robot_data->plot1.clear();
  robot_data->plot2.clear();
 // gl_area->queue_render();
 */
  gl_model_area->queue_render();

  return true;
}

bool Gui::onKeyPress(GdkEventKey* event) {
  std::cout << event->keyval << ' ' << event->hardware_keycode << ' ' << event->state << std::endl;

  plot3.updateModel(event->keyval);
  
  gl_model_area->queue_render();

  return false;
}