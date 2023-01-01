#include <iostream>
#include <fstream>
#include <SDL2/SDL.h>
#include <GL/gl.h>
using namespace std;

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

struct tile_verts
{
    float x,y,z,u,v;
};

struct tile_model
{
    int num_verts;
    tile_verts *vert_data;
};

struct tile_set
{
    int num_tiles;
    tile_model *tiles;
};

struct tile_file_header
{
    char header[4];
    int num_tiles;
};

tile_set *load_model(const char *file_name)
{
    ifstream file(file_name, ios::in | ios::binary);
    if(file.is_open())
    {
        tile_set *tiles = new tile_set;
        // Read file header
        tile_file_header header;
        file.read((char*)&header, sizeof(header));

        tiles->num_tiles = header.num_tiles;
        tiles->tiles = new tile_model[tiles->num_tiles];

#if 0
        cout << "Header " << header.header[0] << header.header[1] << header.header[2] 
             << header.header[3] << endl;
        cout << "Read " << tiles->num_tiles << " " << header.num_tiles << endl;
#endif
        for(int i=0; i < tiles->num_tiles; i++)
        {
            tile_model &current_tile = tiles->tiles[i];
            file.read((char*)&current_tile.num_verts, sizeof(current_tile.num_verts));
            current_tile.vert_data = new tile_verts[current_tile.num_verts];
            file.read((char*)current_tile.vert_data, current_tile.num_verts*sizeof(tile_verts));
        }

        file.close();
        return tiles;
    }
    else
    {
        return NULL;
    }
};

int main()
{
    if(SDL_Init(SDL_INIT_EVERYTHING) == -1)
    {
        return 0;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

    SDL_Window *window = SDL_CreateWindow("Korok Game", 
                                       SDL_WINDOWPOS_CENTERED,
                                       SDL_WINDOWPOS_CENTERED,
                                       WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_OPENGL);

    if(window == NULL)
    {
        cout << "Failed to create window" << endl;
        return 0;
    }

    SDL_GLContext context = SDL_GL_CreateContext(window);

    if(context == NULL)
    {
        cout << "Failed to create context" << endl;
        return 0;
    }

    tile_set *tiles = load_model("data/tiles.bin");

    if(tiles == NULL)
    {
        cout << "Failed to load tiles" << endl;
        return 0;
    }

    bool running = true;
    SDL_Event e;
    while(running)
    {
        while(SDL_PollEvent(&e))
        {
            if(e.type == SDL_QUIT)
            {
                running = false;
            }
        }

        glClear(GL_COLOR_BUFFER_BIT);

        SDL_GL_SwapWindow(window);
    }

    return 0;
}
