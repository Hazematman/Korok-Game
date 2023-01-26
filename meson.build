project('korok_game', 'cpp')

sdl2 = dependency('SDL2')
opengl = dependency('OpenGL')
bullet = dependency('bullet')

srcs = [
    'src/main.cpp',
    'src/model.cpp',
    'src/gameobject.cpp',
    ]

incdir = include_directories('include')

executable('korok_game', srcs, 
    dependencies : [sdl2, opengl, bullet],
    include_directories : incdir)
