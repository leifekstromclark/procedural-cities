SDL_INC = `sdl2-config --cflags`
SDL_LIB = `sdl2-config --libs`

OBJS = planner.o project.o cityLayout.o goal.o util.o singleChannelMap.o
FLAGS = -g
INCLUDE = $(SDL_INC)
LIBS = $(SDL_LIB)


project: $(OBJS)
	g++ $(FLAGS) -o $@ $^ $(LIBS)

%.o: %.cc
	g++ -c $(FLAGS) -o $@ $< $(INCLUDE)

.PHONY: clean

clean:
	rm -f project $(OBJS)
