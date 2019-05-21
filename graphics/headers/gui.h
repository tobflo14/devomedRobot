#ifndef GUI_H
#define GUI_H

#include <gtkmm.h>
#include "global_struct.h"
#include <gtkmm/glarea.h>
#include "plot3d.h"


class Gui {
    public:
        Gui(void *arg);
        void update_btn();
        void update_values();
        void* GuiThread();
        void shutdown();
        void change(double &value_ptr, double value);
        void create_button(std::string name, double &value_ptr, double change_value);
        void draw_triangle();
        static bool gl_draw();
        void onRealize();
        void onUnrealize();
        bool onRender(const Glib::RefPtr<Gdk::GLContext>& /* context */);
        void onRealizeModel();
        void onUnrealizeModel();
        bool onRenderModel(const Glib::RefPtr<Gdk::GLContext>& /* context */);
        bool update_plot();
        bool update_model();
        bool onKeyPress(GdkEventKey* event);
        void updateRobotStatus();
        
        //void create_button(std::string name, double *value_ptr, double change_value, void (Gui::*f)());

    private:
        void init();
        void i_up();
        void i_down();
        void d_up();
        void d_down();

        int current_exercise;
        bool show_debug;
       // shared_robot_data *robot_data1;
        Gtk::Window *mainWindow;
        Gtk::Label  *lbl_user_info;
        
        Gtk::GLArea *gl_model_area;
        Gtk::Button *btn_p_up;
        Gtk::Button *btn_p_down ;
        Gtk::Button *btn_i_up;
        Gtk::Button *btn_i_down;
        Gtk::Button *btn_d_up;
        Gtk::Button *btn_d_down;
        Gtk::Button *btn_shutdown;   
        Gtk::Button *btn_choose_exercise;
        Gtk::Label  *lbl_p_value;
        Gtk::Label  *lbl_i_value;
        Gtk::Label  *lbl_d_value;
        Gtk::Label  *lbl_mass_value;
        
        shared_robot_data *robot_data;

        Gtk::Notebook   *notebook;
        Gtk::Grid       *page_user_grid;
        Gtk::Label      *lbl_page_user;
        Gtk::Image      *image_user1;
        Gtk::Image      *image_user2;
        Gtk::Button     *btn_user1;
        Gtk::Button     *btn_user2;

        Gtk::Grid       *page_program_grid;
        Gtk::ButtonBox  *buttonbox_programs;
        Gtk::ToggleButton     *btn_program1;
        Gtk::ToggleButton     *btn_program2;
        Gtk::ToggleButton     *btn_new_program;
        Gtk::Paned      *paned_program_info;
        Gtk::Box        *box_program;
        Gtk::Entry      *entry_title_program;
        Gtk::Label      *lbl_chosen_program;
        Gtk::Label      *lbl_info_program;
        Gtk::Label      *lbl_exercise_in_program;
        Gtk::Label      *lbl_prog_ex1;
        Gtk::Label      *lbl_prog_ex2;

        Gtk::Grid       *page_exercise;
        Gtk::Label      *lbl_page_exercise;
        Gtk::Entry      *entry_title_exercise;
        Gtk::Button     *btn_new_exercise;
        Gtk::ButtonBox  *buttonbox_exercises;
        Gtk::TreeView   *treeview_exercises;
        Gtk::Button     *btn_exercise1;
        Gtk::Button     *btn_exercise2;
        Gtk::Button     *btn_exercise3;
        Gtk::Paned      *paned_exercise_info;
        Gtk::Label      *lbl_chosen_exercise;
        Gtk::Label      *lbl_info_exercise;
        Gtk::Image      *image_exercise;
        Gtk::ToggleButton     *btn_record;
        Gtk::Button     *btn_save_exercise;

        Gtk::Grid       *page_run_grid;
        Gtk::Label      *lbl_page_run;
        Gtk::GLArea     *gl_area;
        Gtk::Label      *lbl_resistance;
        Gtk::Label      *lbl_robot_status;
        Gtk::Paned      *paned_plot_value;
        Gtk::Button     *btn_screenshot;
        Gtk::Label      *lbl_plot_info;
        Gtk::ComboBox   *combobox_run_exercise;
        Gtk::ToggleButton     *btn_run_exercise;
        Gtk::Label      *lbl_rep_count;
        Gtk::Label      *lbl_current_plot_value;
        Gtk::SpinButton *spin_resistance;
        Gtk::ProgressBar *progress_exercise;




        double p_value;
        double i_value;
        double d_value;
        double mass_value;
        double change_value;
        
        Glib::RefPtr<Gtk::Builder> builder;

        //Gtk::Builder builder2;

        char p_value_char[30] = {};
        char i_value_char[30] = {};
        char d_value_char[30] = {};
        char mass_value_char[30] = {};
        char lbl_plot_value[30] = {};

        GtkWidget *gl_drawing_area;

};

#endif