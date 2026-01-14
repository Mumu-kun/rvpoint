# Compiler warnings configuration
# Usage: include(cmake/CompilerWarnings.cmake)

function(set_project_warnings target_name)
    set(GCC_CLANG_WARNINGS
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
        -Wconversion
        -Wsign-conversion
        -Wdouble-promotion
        -Wformat=2
    )

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        set(GCC_WARNINGS
            ${GCC_CLANG_WARNINGS}
            -Wmisleading-indentation
            -Wduplicated-cond
            -Wduplicated-branches
            -Wlogical-op
            -Wnull-dereference
        )
        set(PROJECT_WARNINGS ${GCC_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(PROJECT_WARNINGS ${GCC_CLANG_WARNINGS})
    endif()

    target_compile_options(${target_name} INTERFACE ${PROJECT_WARNINGS})
endfunction()
