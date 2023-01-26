#include <iostream>
#include <fstream>
#include <vector>
#include <stdint.h>
#include <SDL2/SDL.h>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletDynamics/Character/btKinematicCharacterController.h>
#include <bullet/BulletCollision/CollisionDispatch/btGhostObject.h>
#include <bullet/BulletCollision/Gimpact/btGImpactShape.h>

#define JC_VORONOI_IMPLEMENTATION
#include <jc_voronoi.h>
using namespace std;

#define WINDOW_WIDTH 1024
#define WINDOW_HEIGHT 768

#define NUM_POINTS 128

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

GLuint build_tile_mesh(map_grid &grid, tile_set &set, vector<glm::vec3> &verts)
{
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
                    verts.push_back(glm::vec3(vert));
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

#define MAX_RELAXATIONS 2

/* Code copied from jcash voronoi example application */
/* https://github.com/JCash/voronoi/blob/dev/src/main.c */
static void relax_points(const jcv_diagram* diagram, jcv_point* points)
{
    const jcv_site* sites = jcv_diagram_get_sites(diagram);
    for( int i = 0; i < diagram->numsites; ++i )
    {
        const jcv_site* site = &sites[i];
        jcv_point sum = site->p;
        int count = 1;

        const jcv_graphedge* edge = site->edges;

        while( edge )
        {
            sum.x += edge->pos[0].x;
            sum.y += edge->pos[0].y;
            ++count;
            edge = edge->next;
        }

        points[site->index].x = sum.x / (jcv_real)count;
        points[site->index].y = sum.y / (jcv_real)count;
    }
}

jcv_point points[NUM_POINTS];
void generate_voronoi_map()
{
    srand(0);

    for(int i = 0; i < NUM_POINTS; i++)
    {
        points[i].x = (float)(rand() / (float)RAND_MAX) * 16.0f;
        points[i].y = (float)(rand() / (float)RAND_MAX) * 16.0f;
    }

    for(int i = 0; i < MAX_RELAXATIONS; i++)
    {
        jcv_diagram diagram = {0};
        jcv_diagram_generate(NUM_POINTS, (const jcv_point*)points, NULL, NULL, &diagram);
        relax_points(&diagram, points);
        jcv_diagram_free(&diagram);
    }

    jcv_diagram diagram = {0};
    jcv_diagram_generate(NUM_POINTS, (const jcv_point*)points, NULL, NULL, &diagram);
}

float cube_verts[] =
{
    // Back face
    -1.0f, -1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f,  1.0f, -1.0f,
    -1.0f, -1.0f, -1.0f,
     1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,

    // Front face
    -1.0f, -1.0f,  1.0f,
     1.0f, -1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
    -1.0f, -1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f,

    // Right face
     1.0f, -1.0f, -1.0f,
     1.0f, -1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f,  1.0f,  1.0f,
     1.0f,  1.0f, -1.0f, 

    // Left face
    -1.0f, -1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f,
    -1.0f, -1.0f, -1.0f,
    -1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f, -1.0f, 

    // Top face
    -1.0f,  1.0f, -1.0f,
     1.0f,  1.0f, -1.0f,
     1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f, -1.0f,
     1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f,

    // Top face
    -1.0f, -1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f, -1.0f,  1.0f,
    -1.0f, -1.0f, -1.0f,
     1.0f, -1.0f,  1.0f,
    -1.0f, -1.0f,  1.0f,
};

void draw_cube()
{
    glBegin(GL_TRIANGLES);
    for(size_t i = 0; i < sizeof(cube_verts)/sizeof(cube_verts[0]); i += 3)
    {
        glVertex3f(cube_verts[i], cube_verts[i+1], cube_verts[i+2]);
    }
    glEnd();
}

class CharacterController
{
private:
    btPairCachingGhostObject *ghost;
    btBoxShape *shape;
    btKinematicCharacterController *con;
public:
    CharacterController(btDiscreteDynamicsWorld *dynamics_world);

    void get_camera_matrix(glm::mat4 &view_mat);

    void update(float dt);
    void draw();
};

CharacterController::CharacterController(btDiscreteDynamicsWorld *dynamics_world)
{
    ghost = new btPairCachingGhostObject();
    shape = new btBoxShape(btVector3{1.0f, 1.0f, 1.0f});

    con = new btKinematicCharacterController(ghost, shape, btScalar(1.0f), btVector3{0.0f, 1.0f, 0.0f});

    con->warp(btVector3{5.0f, 5.0f, -5.0f});

    ghost->setCollisionShape(shape);
    //ghost->setWorldTransform(start);
    ghost->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
    

    dynamics_world->addCollisionObject(ghost, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter|btBroadphaseProxy::DefaultFilter);
    dynamics_world->addAction(con);
}

void CharacterController::get_camera_matrix(glm::mat4 &view_mat)
{
    btTransform trans = ghost->getWorldTransform();
    btVector3 position = trans.getOrigin();

    btVector3 forward = trans.getBasis().getColumn(2);

    btVector3 camera_pos = position + btVector3{0.0f, 3.0f, 0.0f} - 5.0f*forward;
    glm::vec3 cam_pos_glm(camera_pos[0], camera_pos[1], camera_pos[2]);
    glm::vec3 target_pos(position[0], position[1], position[2]);
    view_mat = glm::lookAt(cam_pos_glm, target_pos, glm::vec3(0.0f, 1.0f, 0.0f)); 
}

void CharacterController::update(float dt)
{
    float speed = 1.0f;
    float turn_speed = 1.0f;
    btTransform trans = ghost->getWorldTransform();
    btVector3 direction{0,0,0}; 

    float turn_direction = 0.0f;

    const uint8_t *state = SDL_GetKeyboardState(NULL);
    if(state[SDL_SCANCODE_LEFT])
    {
        turn_direction = 1.0f;
    }
    else if(state[SDL_SCANCODE_RIGHT])
    {
        turn_direction = -1.0f;
    }

    btTransform rot = btTransform(btQuaternion(btVector3{0.0f, 1.0f, 0.0f}, dt*turn_direction*turn_speed));
    btTransform new_trans = ghost->getWorldTransform() * rot;
    ghost->setWorldTransform(new_trans);
#if 0
    btMatrix3x3 orientation = ghost->getWorldTransform().getBasis();
    orientation *= btMatrix3x3(btQuaternion(btVector3(0.0f, 1.0f, 0.0f), dt*turn_direction*turn_speed));
    ghost->getWorldTransform().setBasis(orientation);
    ghost->setWorldTransform(new_trans);
#endif
    
    if(state[SDL_SCANCODE_UP])
    {
        direction[2] = -1.0f;
    }
    else if(state[SDL_SCANCODE_DOWN])
    {
        direction[2] = 1.0f;
    }

    btVector3 walk_direction = -direction[2] * trans.getBasis().getColumn(2);
    cout << "Walk: " << walk_direction[0] << " " << walk_direction[1] << " " << walk_direction[2] << endl;
    con->setWalkDirection(dt*speed*walk_direction);
}

void CharacterController::draw()
{
    btTransform trans = ghost->getWorldTransform();
    btVector3 pos = trans.getOrigin();
    cout << "Player: " << pos[0] << " " << pos[1] << " " << pos[2] << endl;
    float mat[16];
    trans.getOpenGLMatrix(mat);
    glPushMatrix();
    glMultMatrixf(mat);
    glColor3f(0.0f, 0.0f, 1.0f);
    draw_cube();

    glPopMatrix();

    btVector3 new_pos = pos + 2.0f*trans.getBasis().getColumn(0);
    btVector3 new_pos2 = pos + 2.0f*trans.getBasis().getColumn(1);
    btVector3 new_pos3 = pos + 2.0f*trans.getBasis().getColumn(2);

    glBegin(GL_LINES);
        glColor3f(1.0f, 1.0f, 0.0f);
        glVertex3f(pos[0], pos[1], pos[2]);
        glVertex3f(new_pos[0], new_pos[1], new_pos[2]);

        glColor3f(0.0f, 1.0f, 1.0f);
        glVertex3f(pos[0], pos[1], pos[2]);
        glVertex3f(new_pos2[0], new_pos2[1], new_pos2[2]);

        glColor3f(1.0f, 0.0f, 1.0f);
        glVertex3f(pos[0], pos[1], pos[2]);
        glVertex3f(new_pos3[0], new_pos3[1], new_pos3[2]);
    glEnd(); 
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

    generate_voronoi_map();

    map_grid *map = build_map();
    vector<glm::vec3> terrain_verts;
    GLuint map_list = build_tile_mesh(*map, *tiles, terrain_verts); 

    // Initalize physics
    btDefaultCollisionConstructionInfo constructionInfo = btDefaultCollisionConstructionInfo();
    constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize = 512;
    constructionInfo.m_defaultMaxPersistentManifoldPoolSize = 512;
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration(constructionInfo);
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    btBroadphaseInterface* overlappingPairCache = new btAxisSweep3(btVector3{-100,-100,-100}, btVector3{100,100,100});
    overlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3{0, -20, 0});dynamicsWorld->setGravity(btVector3{0, -20, 0});

    btTriangleMesh *trimesh = new btTriangleMesh();
    btVector3 verts[3];
    // Build collision mesh for world
    for(size_t i = 0; i < terrain_verts.size(); i++)
    {
        glm::vec3 &vert = terrain_verts[i];
        size_t tri_index = i % 3;
        verts[tri_index] = btVector3{vert.x, vert.y, vert.z};

        if(tri_index == 2)
        {
            trimesh->addTriangle(verts[0], verts[1], verts[2]);
        }
    }

    {
        btGImpactMeshShape *gimpact = new btGImpactMeshShape(trimesh);
        gimpact->updateBound();
        btCollisionShape *col_shape = gimpact;
        btDefaultMotionState *motion_state = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3{0.0f, 0.0f, 0.0f}));
        btVector3 inertia;
        col_shape->calculateLocalInertia(0, inertia);
        btRigidBody::btRigidBodyConstructionInfo ci(0, motion_state, col_shape, inertia);
        btRigidBody *body = new btRigidBody(ci);

        dynamicsWorld->addRigidBody(body);
    }

    btVector3 extents(1.0f, 1.0f, 1.0f);
    btVector3 origin(3.0f, 100.0f, -5.0f);
    float mass = 1.0f;
    btCollisionShape *col_shape = new btBoxShape(extents);
    btDefaultMotionState *motion_state = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), origin));
    btVector3 inertia;
    col_shape->calculateLocalInertia(mass, inertia);
    btRigidBody::btRigidBodyConstructionInfo ci(mass, motion_state, col_shape, inertia);
    btRigidBody *body = new btRigidBody(ci);
    dynamicsWorld->addRigidBody(body);

    CharacterController player(dynamicsWorld);

    glm::mat4 perspective = glm::perspectiveFov(90.0f, (float)WINDOW_WIDTH, (float)WINDOW_HEIGHT, 0.1f, 100.0f);

    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(&perspective[0][0]);

    glm::vec3 pos(0.0f, 0.0f, 0.0f);

    const Uint8* state = SDL_GetKeyboardState(NULL);

    bool running = true;
    SDL_Event e;
    uint64_t start = SDL_GetTicks64();
    uint64_t end = start;
    float dt = 1.0f / 60.0f;
    while(running)
    {
        while(SDL_PollEvent(&e))
        {
            if(e.type == SDL_QUIT)
            {
                running = false;
            }
        }

        player.update(dt);

        dynamicsWorld->stepSimulation(dt, 2);

        glm::mat4 look_at;
        player.get_camera_matrix(look_at);
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(&look_at[0][0]);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        {
            btTransform trans = body->getWorldTransform();

            glm::mat4 t_mat(1.0f);
            trans.getOpenGLMatrix(&t_mat[0][0]);
            glPushMatrix();
            glMultMatrixf(&t_mat[0][0]);
            glColor3f(0.0f, 1.0f, 0.0f);
            draw_cube();
            glPopMatrix();
        }

        player.draw();

        glColor3f(1.0f, 0.0f, 0.0f);
        glCallList(map_list);

        SDL_GL_SwapWindow(window);
        end = SDL_GetTicks64();
        dt = (float)(end - start)/1000.0f;
        start = end;
    }

    return 0;
}
