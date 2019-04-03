#include <gtkmm>
#include <string.h>

class Button {
    public:
        Button(std::string button1);
        build_button(Gtk::Builder builder);
        std::string name;
        Gtk::Button btn;

}