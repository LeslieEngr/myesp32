set(srcs "main.cpp"
                "driver/i2cdev/i2cdev.cpp"
                "driver/pca9555/pca9555.cpp")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       REQUIRES fatfs sd_card
                       WHOLE_ARCHIVE)
