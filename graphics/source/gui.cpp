#include "gui.h"
#include "global_struct.h"
#include <gtkmm.h>
#include <string.h>
#include <gtkmm/glarea.h>
#include "plot2d.h"
//#include "plot3d.h"

//shared_robot_data *robot_data;
//auto app

//
Plot2d plot = Plot2d(1000,2,2); 
Plot3d plot3 = Plot3d();


 Gui::Gui(void *arg) {
    this->robot_data = (shared_robot_data *)arg;
    show_debug = false;
    Gui::init();
 }

 void Gui::init() {
    
     
    // this->plot3 = Plot3d();
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
    this->builder = Gtk::Builder::create_from_file("window_main2.glade");

    //Add css to the gui
    Glib::RefPtr<Gtk::CssProvider> cssProvider = Gtk::CssProvider::create();
    cssProvider->load_from_path("gui.css");
	  Glib::RefPtr<Gtk::StyleContext> styleContext = Gtk::StyleContext::create();
	  Glib::RefPtr<Gdk::Screen> screen = Gdk::Screen::get_default();
	  styleContext->add_provider_for_screen(screen, cssProvider, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);


    //auto app
    p_value = robot_data->kp;
    i_value = robot_data->ki;
    d_value = robot_data->kd;
    mass_value = robot_data->fake_mass;

    builder->get_widget("window_main", mainWindow);
    mainWindow->fullscreen();

    builder->get_widget("btn_record", btn_record);
    builder->get_widget("lbl_p_value", lbl_p_value);
    builder->get_widget("lbl_i_value", lbl_i_value);
    builder->get_widget("lbl_d_value", lbl_d_value);
    builder->get_widget("lbl_mass_value", lbl_mass_value);
    builder->get_widget("gl_area", gl_area);
    builder->get_widget("gl_model_area", gl_model_area);
    builder->get_widget("lbl_current_plot_value", lbl_current_plot_value);
    builder->get_widget("btn_save_exercise", btn_save_exercise);
    //builder->get_widget("btn_choose_exercise", btn_choose_exercise);

    create_button("btn_p_up", p_value, 0.0001);
    create_button("btn_p_down", p_value, -0.0001);
    create_button("btn_i_up", i_value, 0.001);
    create_button("btn_i_down", i_value, -0.001);
    create_button("btn_d_up", d_value, 0.00001);
    create_button("btn_d_down", d_value, -0.00001);
    create_button("btn_mass_up", mass_value, 0.1);
    create_button("btn_mass_down", mass_value, -0.1);
  
    builder->get_widget("notebook", notebook);
    notebook->get_nth_page(1)->hide();
    notebook->get_nth_page(2)->hide();
    notebook->get_nth_page(3)->hide();
    notebook->get_nth_page(4)->hide();
    notebook->get_nth_page(5)->hide();

    builder->get_widget("lbl_user_info", lbl_user_info);

    builder->get_widget("btn_user1", btn_user1);
    btn_user1->signal_clicked().connect([&]() {
      lbl_user_info->show();
      notebook->get_nth_page(1)->show();
      notebook->get_nth_page(2)->show();
      notebook->get_nth_page(3)->show();
      notebook->set_current_page(1);
    });

    builder->get_widget("entry_title_program", entry_title_program);
    entry_title_program->set_text("");
    entry_title_program->set_placeholder_text("Navn på det nye programmet");

    builder->get_widget("box_program", box_program);
    box_program->hide();

    builder->get_widget("btn_new_program", btn_new_program);
    
    builder->get_widget("btn_program1", btn_program1);
    btn_program1->signal_clicked().connect([&]() {
      btn_new_program->set_active(false);
      entry_title_program->set_text("Program 1");
      box_program->show();
    });


/*
    builder->get_widget("buttonbox_programs", buttonbox_programs);
    for (auto row: buttonbox_programs->get_children()) {
      builder->get_widget("btn_program1", btn_program1);
      row->id->signal_clicked().connect([&]() {
        row->activate();
      });;
    }
  *//*
    builder->get_widget("treeview_exercises", treeview_exercises);
    Glib::RefPtr<Gtk::TreeModel> treeview_exercises_model = treeview_exercises->get_model();
    Glib::RefPtr<Gtk::TreeSelection> treeview_exercises_selected = treeview_exercises->get_selection();
    Gtk::TreeModel::iterator iter = treeview_exercises_selected->get_selected();
    Gtk::TreeModelColumn<Glib::ustring> m_col_text;
    if(iter) {
      Gtk::TreeModel::Row row = *iter;
      std::cout << treeview_exercises->get_column(1)->get_title() << endl;
    }*/

    builder->get_widget("progress_exercise", progress_exercise);

    btn_record->signal_clicked().connect([&]() {
      if (btn_record->get_active()) {
        robot_data->tracking_data.clear();
        robot_data->track_position = true;
        btn_record->set_label("Stopp tracking");
        std::cout << "Starting tracking" << std::endl;
      } else {
        robot_data->track_position = false;
        btn_record->set_label("Start tracking");
        std::cout << "Stopping tracking" << std::endl;
      }
    });

    builder->get_widget("entry_title_exercise", entry_title_exercise);

    builder->get_widget("btn_run_exercise", btn_run_exercise);
    btn_run_exercise->set_sensitive(false);
    btn_run_exercise->signal_clicked().connect([&]() {
      if (btn_run_exercise->get_active()) {
        robot_data->floating_mode = false;
        btn_run_exercise->set_label("Stopp øvelse");
      } else {
        robot_data->floating_mode = true;
        btn_run_exercise->set_label("Start øvelse");
      }
    });

    builder->get_widget("combobox_run_exercise", combobox_run_exercise);
    combobox_run_exercise->signal_changed().connect([&](){
      current_exercise = std::stoi(combobox_run_exercise->get_active_id());
      std::string openfile;
      if (current_exercise == 1) {
        openfile = "exercises/fremoverbøy.csv";
      }
      if (current_exercise == 4) {
        openfile = "exercises/bue.csv";
      }
      if (!openfile.empty()) {
        robot_data->track_path = readMatrix(openfile);
        btn_run_exercise->set_sensitive(true);
        std::cout << "File opened " << openfile << std::endl;
      }
    });


    btn_save_exercise->signal_clicked().connect([&]() {
      btn_save_exercise->set_label("Lagrer...");
      std::stringstream outputfile;
      outputfile << "exercises/" << entry_title_exercise->get_text() << ".csv";
      //Save tracking data.
      std::ofstream file_to_write;
      std::cout << "Saving tracking data..." << std::endl;
      try {
          file_to_write.open(outputfile.str());
          for (size_t i = 0; i < robot_data->tracking_data.size(); i += 10) {
              file_to_write << robot_data->tracking_data[i][0] << ";";
              file_to_write << robot_data->tracking_data[i][1] << ";";
              file_to_write << robot_data->tracking_data[i][2] << ";\n";
          }
          file_to_write.close();
      } catch (std::system_error& e) {
          std::cerr << "Error opening file: " << e.what() << std::endl;
      }
      std::cout << "Tracking data saved." << std::endl;
      btn_save_exercise->set_label("Lagret!");
    });

    builder->get_widget("lbl_robot_status", lbl_robot_status);

    Glib::signal_timeout().connect([&]() -> bool {
      progress_exercise->set_fraction(robot_data->fractionCompleted);
      updateRobotStatus();
      return true;
    }, 40);

    builder->get_widget("spin_resistance", spin_resistance);
    spin_resistance->signal_value_changed().connect([&](){
      robot_data->wanted_force = spin_resistance->get_value_as_int();
    });

    
    // GL PLOT AREA   

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

void Gui::updateRobotStatus() {
  lbl_robot_status->set_text(std::to_string((int)robot_data->robot_mode));
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
  //robot_data->plot1.push_back(Point(value1, value1));
  robot_data->plot2.push_back(Point(value2, 0));
  size_t i;
  for (i = 0; i < robot_data->plot1.size(); i++) {
  //if (robot_data->plot1.size() > 0) {
      Point values[] = {robot_data->plot1[i], robot_data->plot2[0]};
      plot.graph_update(values);
  }
 // }
 if (robot_data->plot1.size() > 0) {
  sprintf(lbl_plot_value, "%.2f", robot_data->plot1[i].second);
 }
  lbl_current_plot_value->set_text(lbl_plot_value);
  robot_data->plot1.clear();
  robot_data->plot2.clear(); 
  gl_area->queue_render();
  return true;
}

bool Gui::update_model() {
  gl_model_area->queue_render();
  return true;
}

bool Gui::onKeyPress(GdkEventKey* event) {
  std::cout << event->keyval << ' ' << event->hardware_keycode << ' ' << event->state << std::endl;
  //Close window on Esc press
  if (event->keyval == 65307) {
    robot_data->shutdown = true;
    mainWindow->close();
  }
  if (event->keyval == 100) {
    if (show_debug) {
      notebook->get_nth_page(4)->hide();
      notebook->get_nth_page(5)->hide();
      show_debug = false;
    } else {
      notebook->get_nth_page(4)->show();
      notebook->get_nth_page(5)->show();
      show_debug = true;
    }
  }

  plot3.updateModel(event->keyval);
  
  gl_model_area->queue_render();

  return false;
}