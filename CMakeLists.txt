add_falcor_executable(Bezier)

target_sources(Bezier PRIVATE
    Bezier.cpp
    Bezier.h
    Shaders/bezier2d.ps.slang
    Shaders/bezier2d.vs.slang
    Shaders/raymarch.slang
    Shaders/bezier_eval.slang
)

target_copy_shaders(Bezier Samples/Bezier)

target_source_group(Bezier "Samples")

find_package (Eigen3 3.4 REQUIRED NO_MODULE)
link_libraries (Bezier Eigen3::Eigen)