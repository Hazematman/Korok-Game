#include <iostream>
#include <fstream>
#include <vector>
#include <stdint.h>
#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace std;

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

struct tile_vert
{
    float x,y,z,u,v;
};

struct tile_model
{
    int num_verts;
    tile_vert *vert_data;
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
            current_tile.vert_data = new tile_vert[current_tile.num_verts];
            file.read((char*)current_tile.vert_data, current_tile.num_verts*sizeof(tile_vert));
        }

        file.close();
        return tiles;
    }
    else
    {
        return NULL;
    }
};

struct map_cell
{
    int rotation;
    int type;
};

struct map_grid
{
    int x_size, y_size, z_size;
    map_cell *cells;
};

GLuint build_tile_mesh(map_grid &grid, tile_set &set)
{
    vector<glm::vec4> verts;
    for(int z = 0; z < grid.z_size; z++)
    {
        for(int y = 0; y < grid.y_size; y++)
        {
            for(int x = 0; x < grid.x_size; x++)
            {
                map_cell &cell = grid.cells[z*grid.y_size*grid.x_size + y*grid.x_size + x];
                if(cell.type == -1)
                {
                    continue;
                }

                tile_model &model = set.tiles[cell.type];

                glm::mat4 rotation = glm::eulerAngleYXZ(glm::radians((float)cell.rotation), 0.0f, 0.0f);
                glm::mat4 translation = glm::translate(glm::mat4(1.0f), glm::vec3((float)x, (float)y-0.5f, (float)-z));
                glm::mat4 transform = translation*rotation;
                for(int i=0; i < model.num_verts; i++)
                {
                    tile_vert &tvert = model.vert_data[i];
                    glm::vec4 vert(tvert.x, tvert.y, tvert.z, 1.0f);
                    vert = transform*vert;
                    verts.push_back(vert);
                }
            }
        }
    }

    GLuint display_list = glGenLists(1);

    glNewList(display_list, GL_COMPILE);

    glBegin(GL_TRIANGLES);
    for(size_t i = 0; i < verts.size(); i++)
    {
        glVertex3f(verts[i].x, verts[i].y, verts[i].z);
    }
    glEnd();
    glEndList();

    cout << "Made mesh with " << verts.size() << " verts" << endl;

    return display_list;
}

map_grid *build_map()
{
    map_grid *map = new map_grid;
    map->z_size = 10;
    map->y_size = 3;
    map->x_size = 10;

    map->cells = new map_cell[map->z_size*map->y_size*map->x_size];

    for(int z = 0; z < map->z_size; z++)
    {
        for(int y = 0; y < map->y_size; y++)
        {
            for(int x = 0; x < map->x_size; x++)
            {
                map_cell &cell = map->cells[z*map->y_size*map->x_size + y*map->x_size + x];
                cell.rotation = 0;
                if(y != 0)
                {
                    cell.type = -1;
                }
                else
                {
                    cell.type = 0;
                }
            }
        }
    }

    return map;
}

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

    map_grid *map = build_map();
    GLuint map_list = build_tile_mesh(*map, *tiles); 

    glm::mat4 perspective = glm::perspectiveFov(90.0f, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(&perspective[0][0]);

    glm::vec3 pos(0.0f, 0.0f, 0.0f);

    const Uint8* state = SDL_GetKeyboardState(NULL);

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

        if(state[SDL_SCANCODE_LEFT])
        {
            pos.x -= 1.0f;
        }
        else if(state[SDL_SCANCODE_RIGHT])
        {
            pos.x += 1.0f;
        }
        
        if(state[SDL_SCANCODE_UP])
        {
            pos.z -= 1.0f;
        }
        else if(state[SDL_SCANCODE_DOWN])
        {
            pos.z += 1.0f;
        }

        glm::mat4 translate = glm::translate(glm::mat4(1.0f), -1.0f*pos);
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(&translate[0][0]);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glColor3f(1.0f, 0.0f, 0.0f);

        glCallList(map_list);

        SDL_GL_SwapWindow(window);
    }

    return 0;
}
