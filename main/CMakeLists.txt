idf_component_register(SRCS "uln2003.c"
                            "ydlidar.c"
                            "sdCard.c"
                            "server/network.c"
                            "server/dns_server.c"
                            "server/rest_server.c"
                            "main.c"
                    INCLUDE_DIRS "")

if(CONFIG_EXAMPLE_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/front/web-demo")
    if(EXISTS ${WEB_SRC_DIR}/dist)
        spiffs_create_partition_image(www ${WEB_SRC_DIR}/dist FLASH_IN_PROJECT)
    else()
        message(FATAL_ERROR "${WEB_SRC_DIR}/dist doesn't exit. Please run 'npm run build' in ${WEB_SRC_DIR}")
    endif()
endif()