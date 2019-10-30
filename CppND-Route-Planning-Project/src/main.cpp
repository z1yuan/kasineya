#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    /*
    float start_x, start_y, end_x, end_y;
    std::cout<<"The map coordinates start from (0,0) around the bottom left and end at (100,100) around the top right." << "\n";
    std::cout<<"Enter start_x [0..100] : ";
    std::cin>>start_x;
    while (!(std::cin) || start_x < 0 || start_x > 100) {
        std::cout << "Invalid entry. Enter a start_x from 0 to 100: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> start_x;
    }
    std::cout<<"Enter start_y [0..100] : ";
    std::cin>>start_y;
    while (!(std::cin) || start_y < 0 || start_y > 100) {
        std::cout << "Invalid entry. Enter a start_y from 0 to 100: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> start_y;
    }
    std::cout<<"Enter end_x [0..100] : ";
    std::cin>>end_x;
    while (!(std::cin) || end_x < 0 || end_x > 100) {
        std::cout << "Invalid entry. Enter a end_x from 0 to 100: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> end_x;
    }
    std::cout<<"Enter end_y [0..100] : ";
    std::cin>>end_y;
    while (!(std::cin) || end_y < 0 || end_y > 100) {
        std::cout << "Invalid entry. Enter a end_y from 0 to 100: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin >> end_y;
    }

    
    std::cout<<"\nStart coordinates: (" << start_x <<", " << start_y<< ")\n";
    std::cout<<"End coordinates: (" << end_x <<", " << end_y<< ")\n"; */

    std::array<float, 4> coordinates;
        int counter = 0;
        float temp = 0;
        while (counter < 4){
            std::cout << "Enter coordinate " << counter + 1 << ": ";
            std::cin >> temp;
            if ( !std::cin.fail() && temp <= 100 && temp >=0){
                coordinates[counter] = temp;
                counter++;
            } else {
                std::cout << "Invalid input" << "\n";
                std::cin.clear();
                std::cin.ignore(32767,'\n');
            }  
        }
    std::cout<<"\nStart coordinates: (" << coordinates[0] <<", " << coordinates[1]<< ")\n";
    std::cout<<"End coordinates: (" << coordinates[2] <<", " << coordinates[3]<< ")\n";
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    //RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    RoutePlanner route_planner{model, coordinates[0], coordinates[1], coordinates[2], coordinates[3]};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
