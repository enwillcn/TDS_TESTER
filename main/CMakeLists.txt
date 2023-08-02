file(GLOB_RECURSE IMAGE_SOURCES images/*.c)

idf_component_register(

    SRC_DIRS
    "."
    "images"
    
       
    INCLUDE_DIRS
    "."
    "include"

)
