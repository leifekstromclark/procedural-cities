#include <SDL.h>
#include <iostream>
#include <string>
#include "cityLayout.h"
#include "vect2.h"

class View {
    SDL_Renderer *renderer;
  public:
    View(SDL_Renderer *renderer): renderer{renderer} {}
    void drawCity(const CityLayout &city) {
        //Clear screen
        SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xFF);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
        for (size_t i = 0; i < city.numRoads(); ++i) {
            Vect2 a = city.nodePos(city.tip(city.roadEdge(i)));
            Vect2 b = city.nodePos(city.root(city.roadEdge(i)));
            SDL_RenderDrawLine(renderer, int(a.getX()), int(a.getY()), int(b.getX()), int(b.getY()));
        }

        for (size_t i = 0; i < city.numBlocks(); ++i) {
            SDL_SetRenderDrawColor(renderer, std::rand() % 156 + 100, std::rand() % 156 + 100, std::rand() % 156 + 100, 0xFF);
            std::vector<Vect2> points = city.blockPoly(i);
            for (size_t j = 0; j < points.size(); ++j) {
                size_t inc = (j + 1) % points.size();
                SDL_RenderDrawLine(renderer, int(points[j].getX()), int(points[j].getY()), int(points[inc].getX()), int(points[inc].getY()));
            }
        }

        //Update screen
        SDL_RenderPresent(renderer);
    }
};

class Demo {
    SDL_Window *window;
    bool running;
    City city;

    void processInput() {
        SDL_Event evnt;
        while (SDL_PollEvent(&evnt)) {
            switch (evnt.type) {
                case SDL_QUIT:
                    running = false;
                //case SDL_MOUSEMOTION:
                    //std::cout << evnt.motion.x << " " << evnt.motion.y << std::endl;
            }
        }
    }
  public:
    Demo(): window{nullptr}, running{true}, city{CityLayout{Vect2(500, 400), Vect2(550, 400), 20, 20, M_PI / 6}} {}
    void play() {
        SDL_InitSubSystem(SDL_INIT_EVENTS);
        SDL_InitSubSystem(SDL_INIT_VIDEO);
        window = SDL_CreateWindow("City Test", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1280, 720, SDL_WINDOW_SHOWN);
        View view = View(SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED));

        std::string cmd;
        while (running) {
            processInput();
            view.drawCity(city);

            std::cin >> cmd;
            if (cmd == "ex") {
                size_t n;
                int x, y;
                std::cin >> n >> x >> y;
                city.extendNode(n, Vect2(x, y) , true);

            } else if (cmd == "con") {
                size_t a, b;
                std::cin >> a >> b;
                city.connectNodes(a, b, true);
            } else if (cmd == "query") {
                std::string what;
                size_t i;
                std::cin >> what >> i;
                if (what == "city") {
                    std::cout << "Num Nodes: " << city.numNodes() << std::endl;
                    std::cout << "Num Roads: " << city.numRoads() << std::endl;
                    std::cout << "Num Blocks: " << city.numBlocks() << std::endl;
                } else if (what == "edge") {
                    std::cout << "Twin: " << city.twin(i) << std::endl;
                    std::cout << "Next: " << city.next(i) << std::endl;
                    std::cout << "Prev: " << city.prev(i) << std::endl;
                    std::cout << "Road: " << city.road(i) << std::endl;
                    std::cout << "Tip: " << city.tip(i) << std::endl;
                    std::optional<size_t> block = city.block(i);
                    if (block) {
                        std::cout << "Block: " << *block << std::endl;
                    } else {
                        std::cout << "Outskirts" << std::endl;
                    }
                } else if (what == "road") {
                    std::cout << "Edge: " << city.roadEdge(i) << std::endl;
                } else if (what == "node") {
                    std::cout << "Edge: " << city.nodeEdge(i) << std::endl;
                    std::cout << "Degree: " << city.degree(i) << std::endl;
                } else if (what == "block") {
                    std::cout << "Edge: " << city.blockEdge(i) << std::endl;
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

int main( int argc, char* args[] )
{
	Demo demo;
    demo.play();
    return 0;
}
