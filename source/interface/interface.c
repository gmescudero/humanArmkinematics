
#include "interface.h"
#include "general.h"


static ERROR_CODE sinterface_create_window();
static gboolean sinterface_plot_loop_func(void *data);

ERROR_CODE interface_initialize(int argc, char **argv) {
    dbg_str("%s -> Initializing user interface",__FUNCTION__);
    
    gtk_init(&argc, &argv);

    log_str("\tGTK+ version: %d.%d.%d", gtk_major_version, 
        gtk_minor_version, gtk_micro_version);
    log_str("\tGlib version: %d.%d.%d", glib_major_version,
        glib_minor_version, glib_micro_version);    

    GtkBuilder      *builder;
    GtkWidget       *window_main;
    GtkWidget       *plotting_area;
    GError          *error = NULL;
    int plot_thread_handler;

    builder = gtk_builder_new();

    if( gtk_builder_add_from_file (builder,"glade/interface.glade",&error) == 0){
        err_str( "gtk_builder_add_from_file FAILED %s\n", error->message );
        return RET_ERROR;
    }

    window_main = GTK_WIDGET(gtk_builder_get_object(builder, "window_main"));
    g_signal_connect(window_main, "destroy", G_CALLBACK(gtk_main_quit), NULL);  

    plotting_area = GTK_WIDGET(gtk_builder_get_object(builder, "plot_area"));
    if (NULL == plotting_area) return RET_ERROR;
    plot_thread_handler = gdk_threads_add_timeout_full(0, 50, sinterface_plot_loop_func, plotting_area, NULL);
    // g_signal_connect(G_OBJECT(plotting_area), "draw", G_CALLBACK(on_draw_event), NULL); 

    // Launch window and stick in there
    gtk_widget_show_all(window_main);

    gtk_main();

    g_source_remove(plot_thread_handler);

    return RET_OK;
}

static ERROR_CODE sinterface_create_window() {

    // Gtk window descriptor
    GtkWidget *widget;

    // Define new window
    widget = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    // Configure window
    gtk_window_set_title(GTK_WINDOW(widget), "Main window");
    gtk_window_set_default_size(GTK_WINDOW(widget), 230, 150);
    gtk_window_set_position(GTK_WINDOW(widget), GTK_WIN_POS_CENTER);

    // Set window to be shown
    gtk_widget_show(widget);
    
    // Connect the close button to the destroy procedure
    g_signal_connect(widget, "destroy", G_CALLBACK(gtk_main_quit), NULL);  

    return RET_OK;
}

ERROR_CODE sinterface_on_plotting_area_draw(GtkWidget *widget, cairo_t *cr) {
    dbg_str("%s -> Draw in plotting area",__FUNCTION__);

    GdkWindow *window;
    GdkRectangle da;            // GtkDrawingArea size

    window = gtk_widget_get_window(widget);
    gdk_window_get_geometry(window, &da.x, &da.y, &da.width, &da.height);


    cairo_set_source_rgb (cr, 1.0, 1.0, 1.0); // draw on a white background
    cairo_paint(cr);
    cairo_translate (cr, 0, da.height);       // data starts at left down position
    cairo_set_line_width (cr, 1);
    cairo_set_source_rgb (cr, 0.8, 0.8, 0.8);
    // Draw four horizontal lines
    for(int i=1;i<=101;i=i+25){
        cairo_move_to (cr, 0.0, (-da.height*i/100));
        cairo_line_to (cr, da.width, (-da.height*i/100));
    }

    cairo_stroke (cr);
    cairo_set_line_width (cr, 3.5);



    return RET_OK;
}

static gboolean sinterface_plot_loop_func(void *data) {
    dbg_str("%s -> Draw plot",__FUNCTION__);

    GtkWidget *widget = (GtkWidget*) data;

    gtk_widget_queue_draw(widget);

    return TRUE;
}