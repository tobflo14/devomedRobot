#ifndef GUI_THREAD_1_H
#define GUI_THREAD_1_H

#include <gtkmm.h>
#include "global_struct.h"


class Gui {
    public:
        Gui(void *arg);
        void update_btn();
        void update_values();
        void* GuiThread();
        void shutdown();
        void change(double &value_ptr, double value);
        void create_button(std::string name, double &value_ptr, double change_value);
        //void create_button(std::string name, double *value_ptr, double change_value, void (Gui::*f)());

    private:
        void init();
        void i_up();
        void i_down();
        void d_up();
        void d_down();
        shared_robot_data *robot_data1;
        Gtk::Window *mainWindow;
        Gtk::Button *btn_p_up;
        Gtk::Button *btn_p_down ;
        Gtk::Button *btn_i_up;
        Gtk::Button *btn_i_down;
        Gtk::Button *btn_d_up;
        Gtk::Button *btn_d_down;
        Gtk::Button *btn_shutdown;
        Gtk::Label  *lbl_p_value;
        Gtk::Label  *lbl_i_value;
        Gtk::Label  *lbl_d_value;
        Gtk::Label  *lbl_mass_value;
        Gtk::Label  *lbl_status;
        shared_robot_data *robot_data;
        double p_value;
        double i_value;
        double d_value;
        double mass_value;
        double change_value;
        
        Glib::RefPtr<Gtk::Application> app;
        Glib::RefPtr<Gtk::Builder> builder;

        //Gtk::Builder builder2;

        char p_value_char[30] = {};
        char i_value_char[30] = {};
        char d_value_char[30] = {};
        char mass_value_char[30] = {};


};

#endif