set(req driver)

idf_component_register(
    SRCS SPI_IO_Master.c
    INCLUDE_DIRS .
    REQUIRES ${req}
)