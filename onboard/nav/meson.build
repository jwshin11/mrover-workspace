project('nav', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('onboard_nav', 'main.cpp', 'stateMachine.cpp', 'rover.cpp', 'searcher.cpp', 'pid.cpp', 'utilities.cpp', 'spiral.cpp',
           dependencies : [liblcm],
           install : true)
