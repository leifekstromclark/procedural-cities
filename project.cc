#include <SDL.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <memory>
#include "cityLayout.h"
#include "util.h"
#include "planner.h"
#include "vect2.h"

class View {
    SDL_Renderer *renderer;
    double zoom;
    Vect2 offset;
    int width;
    int height;
    bool onScreen(Vect2 v) const {
        return 0 <= v.getX() && v.getX() <= width && 0 <= v.getY() && v.getY() <= height;
    }
    void drawLine(Vect2 v1, Vect2 v2, int r, int g, int b) {
        v1 = (v1 + offset).scale(zoom);
        v2 = (v2 + offset).scale(zoom);
        if (onScreen(v1) && onScreen(v2)) {
            SDL_SetRenderDrawColor(renderer, r, g, b, 0xFF);
            SDL_RenderDrawLine(renderer, int(v1.getX()), int(v1.getY()), int(v2.getX()), int(v2.getY()));
        }
    }
    void drawPoly(const std::vector<Vect2> &p, int r, int g, int b) {
        if (p.size() > 0) {
            std::unique_ptr<SDL_Point[]> points(new SDL_Point[p.size() + 1]);
            for (size_t j = 0; j < p.size(); ++j) {
                Vect2 v = (p[j] + offset).scale(zoom);
                if (onScreen(v)) {
                    points[j] = SDL_Point {int(v.getX()), int(v.getY())};
                } else {
                    return;
                }
            }
            points[p.size()] = points[0];
            SDL_SetRenderDrawColor(renderer, r, g, b, 0xFF);
            SDL_RenderDrawLines(renderer, points.get(), p.size() + 1);
        }
    }

  public:
    View(SDL_Renderer *renderer, int width, int height): renderer{renderer}, zoom{1.0}, offset{Vect2(0,0)}, width{width}, height{height} {}
    void drawCity(const CityLayout &city) {

        //Clear screen
        SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
        SDL_RenderClear(renderer);

        for (size_t i = 0; i < city.numRoads(); ++i) {
            Vect2 v1 = city.nodePos(city.tip(city.roadEdge(i)));
            Vect2 v2 = city.nodePos(city.root(city.roadEdge(i)));
            if (city.roadIsHighway(i)) {
                drawLine(v1, v2, 0x00, 0x00, 0x00);
            } else {
                drawLine(v1, v2, 100, 100, 100);
            }
        }
        
        for (size_t i = 0; i < city.numBlocks(); ++i) {
            drawPoly(city.blockPoly(i), std::rand() % 156 + 100, std::rand() % 156 + 100, std::rand() % 156 + 100);
        }
        
        
        //Update screen
        SDL_RenderPresent(renderer);
    }
    void enhance(double z) {zoom = z; }
    void pan(Vect2 off) {offset = off; }
};

class Demo {
    SDL_Window *window;
    bool running;
    Planner planner;

    void processInput() {
        SDL_Event evnt;
        while (SDL_PollEvent(&evnt)) {
            switch (evnt.type) {
                case SDL_QUIT:
                    running = false;
                //case SDL_MOUSEMOTION:
                    //std::cout << evnt.motion.x << ", " << evnt.motion.y << std::endl;
            }
        }
    }
  public:
    Demo(double growthSpeed, CityLayout city): window{nullptr}, running{true}, planner{Planner{city, growthSpeed}} {}
    void play() {
        SDL_InitSubSystem(SDL_INIT_EVENTS);
        SDL_InitSubSystem(SDL_INIT_VIDEO);
        int windowWidth = 1280;
        int windowHeight = 720;
        window = SDL_CreateWindow("City Test", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, windowWidth, windowHeight, SDL_WINDOW_SHOWN);
        View view = View(SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED), windowWidth, windowHeight);

        planner.sampleGoals.push_back(std::make_unique<IntersectionRatio>(1.0, 4));
        
        /*
        for (int i = 0; i < 20; ++i) {
            planner.step();
        }*/

        std::string cmd;
        while (running) {
            processInput();
            view.drawCity(planner.getCity());
            
            std::cin >> cmd;
            if (cmd == "useRule") {
                std::string type;
                std::cin >> type;
                if (type == "random") {
                    planner.extensionGoals.push_back(std::make_unique<RandomDeviation>(1.0, 60, 5, M_PI / 60));
                } else if (type == "paris") {
                    planner.extensionGoals.push_back(std::make_unique<Paris>(1.0, Vect2(550, 400), 70, 40));
                } else if (type == "manhattan") {
                    planner.extensionGoals.push_back(std::make_unique<Manhattan>(1.0, M_PI / 6, 70, 40));
                } else if (type == "sanfrancisco") {
                    planner.extensionGoals.push_back(std::make_unique<SanFrancisco>(1.0, std::make_unique<ClampMap>(std::make_unique<PerlinNoiseMap>(Vect2(6000, 6000), 400, 30, 30, 400, *smootherStep), 200, 400), 60, M_PI / 8, 0.2));
                }
            } else if (cmd == "step") {
                planner.step();
            } else if (cmd == "zoom") {
                double z;
                std::cin >> z;
                view.enhance(z);
            } else if (cmd == "pan") {
                double x, y;
                std::cin >> x >> y;
                view.pan(Vect2(x, y));
            } else if (cmd == "query") {
                std::string what;
                size_t i;
                std::cin >> what >> i;
                if (what == "city") {
                    std::cout << "Num Nodes: " << planner.getCity().numNodes() << std::endl;
                    std::cout << "Num Roads: " << planner.getCity().numRoads() << std::endl;
                    std::cout << "Num Blocks: " << planner.getCity().numBlocks() << std::endl;
                } else if (what == "edge") {
                    std::cout << "Twin: " << planner.getCity().twin(i) << std::endl;
                    std::cout << "Next: " << planner.getCity().next(i) << std::endl;
                    std::cout << "Prev: " << planner.getCity().prev(i) << std::endl;
                    std::cout << "Road: " << planner.getCity().road(i) << std::endl;
                    std::cout << "Tip: " << planner.getCity().tip(i) << std::endl;
                    std::optional<size_t> block = planner.getCity().block(i);
                    if (block) {
                        std::cout << "Block: " << *block << std::endl;
                    } else {
                        std::cout << "Outskirts" << std::endl;
                    }
                } else if (what == "road") {
                    std::cout << "Highway: " << planner.getCity().roadIsHighway(i) << std::endl;
                    std::cout << "Edge: " << planner.getCity().roadEdge(i) << std::endl;
                } else if (what == "node") {
                    std::cout << "Edge: " << planner.getCity().nodeEdge(i) << std::endl;
                    std::cout << "Degree: " << planner.getCity().degree(i) << std::endl;
                } else if (what == "block") {
                    std::cout << "Edge: " << planner.getCity().blockEdge(i) << std::endl;
                }
            } else if (cmd == "quit") {
                running = false;
            }
            
        }

        SDL_DestroyWindow(window);
	    SDL_QuitSubSystem(SDL_INIT_EVENTS);
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        SDL_Quit();
    }
};

int main(int argc, char* args[]) {
    //Planner{CityLayout{Vect2(500, 400), Vect2(550, 400), 20, 40, 20, M_PI / 12}, 0.3}
    std::cout << "Enter Growth Speed:" << std::endl;
    double growthSpeed;
    std::cin >> growthSpeed;
    std::cout << "Enter Road Parameters: (Snap Distance, Extend Distance, Minimum Length) (RECCOMEND 20 40 20 FOR DEMO)" << std::endl;
    double snapDist;
    double extendDist;
    double minLength;
    std::cin >> snapDist >> extendDist >> minLength;
	Demo demo(growthSpeed, CityLayout{Vect2(500, 400), Vect2(550, 400), snapDist, extendDist, minLength, M_PI / 9});
    demo.play();
    return 0;
}

