// Enumeration of modbus slave addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1,
    MB_SLAVE_COUNT
};


//____________________________________________________________________________________________________
// Mechanical Pump Wells Slave Definitions:
//____________________________________________________________________________________________________
// Enumeration of all supported CIDs for device
enum {
    MP_ETM = 0,
    MP_ITM,
    MP_JTM,
    MP_SCM,
    MP_HS1
};

// Modbus Dictionary
const mb_parameter_descriptor_t MP_device_parameters[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length,
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    
    { MP_ETM,                           // CID
      STR("MP_ETM"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      0,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },
    
    { MP_ITM,                    // CID
      STR("MP_ITM"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      2,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { MP_JTM,                    // CID
      STR("MP_JTM"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      4,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { MP_SCM,                       // CID
      STR("MP_SCM"),                    // Param Name
      STR("on/off"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_HOLDING,                    // Modbus Reg Type
      0,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { MP_HS1,                       // CID
      STR("MP_HS1"),                    // Param Name
      STR("on/off"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_COIL,                    // Modbus Reg Type
      0,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U8,                    // Data Type
      1,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

};

// Calculate number of parameters in the table
uint16_t num_MP_device_parameters = (sizeof(MP_device_parameters) / sizeof(MP_device_parameters[0]));

//____________________________________________________________________________________________________
// Electro Submersible Pump Wells Slave Definitions:
//____________________________________________________________________________________________________
// Enumeration of all supported CIDs for device
enum {
    ES_ETM = 0,
    ES_ITM,
    ES_ST1,
    ES_TTM,
    ES_WTM,
    ES_WTB,
    ES_ST2,
    ES_YI1,
    ES_YI2,
    ES_SFM,
    ES_PTSB,
    ES_PTDB,
    ES_TTSB,
    ES_TTDB,
    ES_VT,
    ES_SCM,
    ES_YI,
    ES_HS1,
    ES_HS2
};

// Modbus Dictionary
const mb_parameter_descriptor_t ES_device_parameters[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length,
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    
    { ES_ETM,                           // CID
      STR("ES_ETM"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      0,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },
    
    { ES_ITM,                    // CID
      STR("ES_ITM"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      2,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_ST1,                    // CID
      STR("ES_ST1"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      4,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_TTM,                       // CID
      STR("ES_TTM"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      6,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_WTM,                       // CID
      STR("ES_WTM"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      8,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_WTB,                       // CID
      STR("ES_WTB"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      10,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_ST2,                       // CID
      STR("ES_ST2"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      12,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_YI1,                       // CID
      STR("ES_YI1"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      14,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U16,                    // Data Type
      2,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_YI2,                       // CID
      STR("ES_YI2"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      15,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U16,                    // Data Type
      2,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_SFM,                       // CID
      STR("ES_SFM"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      16,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_PTSB,                       // CID
      STR("ES_PTSB"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      18,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_PTDB,                       // CID
      STR("ES_PTDB"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      20,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_TTSB,                       // CID
      STR("ES_TTSB"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      22,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_TTDB,                       // CID
      STR("ES_TTDB"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      24,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_VT,                       // CID
      STR("ES_VT"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                    // Modbus Reg Type
      26,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_SCM,                       // CID
      STR("ES_SCM"),                    // Param Name
      STR("--"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_HOLDING,                    // Modbus Reg Type
      0,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_YI,                       // CID
      STR("ES_YI"),                    // Param Name
      STR("on/off"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_DISCRETE,                    // Modbus Reg Type
      0,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U8,                    // Data Type
      1,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_HS1,                       // CID
      STR("ES_HS1"),                    // Param Name
      STR("on/off"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_COIL,                    // Modbus Reg Type
      0,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U8,                    // Data Type
      1,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { ES_HS2,                       // CID
      STR("ES_HS2"),                    // Param Name
      STR("on/off"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_COIL,                    // Modbus Reg Type
      1,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U8,                    // Data Type
      1,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

};

// Calculate number of parameters in the table
uint16_t num_ES_device_parameters = (sizeof(ES_device_parameters) / sizeof(ES_device_parameters[0]));


//____________________________________________________________________________________________________
// Progressive Cavity Pump Wells Slave Definitions:
//____________________________________________________________________________________________________
// Enumeration of all supported CIDs for device
enum {
    PC_WTM = 0,
    PC_WTB,
    PC_ST2,
    PC_YI1,
    PC_YI2,
    PC_SFM,
    PC_PTSB,
    PC_PTDB,
    PC_TTSB,
    PC_TTDB,
    PC_VT,
    PC_SCM,
    PC_YI,
    PC_HS1
};

// Modbus Dictionary
const mb_parameter_descriptor_t PC_device_parameters[] = {
    // CID, Name, Units, Modbus addr, register type, Modbus Reg Start Addr, Modbus Reg read length,
    // Instance offset (NA), Instance type, Instance length (bytes), Options (NA), Permissions
    
    { PC_WTM,                           // CID
      STR("PC_WTM"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      0,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },
    
    { PC_WTB,                    // CID
      STR("PC_WTB"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      2,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_ST2,                    // CID
      STR("PC_ST2"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      4,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_YI1,                    // CID
      STR("PC_YI1"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      6,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U16,                   // Data Type
      2,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_YI2,                    // CID
      STR("PC_YI2"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      7,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U16,                   // Data Type
      2,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_SFM,                    // CID
      STR("PC_SFM"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      8,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_PTSB,                    // CID
      STR("PC_PTSB"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      10,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_PTDB,                    // CID
      STR("PC_PTDB"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      12,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_TTSB,                    // CID
      STR("PC_TTSB"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      14,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_TTDB,                    // CID
      STR("PC_TTDB"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      16,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_VT,                    // CID
      STR("PC_VT"),                 // Param Name
      STR("--"),                        // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_INPUT,                 // Modbus Reg Type
      18,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                   // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_SCM,                       // CID
      STR("PC_SCM"),                    // Param Name
      STR("on/off"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_HOLDING,                    // Modbus Reg Type
      0,                                // Reg Start
      2,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_FLOAT,                    // Data Type
      4,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_YI,                       // CID
      STR("PC_YI"),                    // Param Name
      STR("on/off"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_DISCRETE,                    // Modbus Reg Type
      0,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U8,                    // Data Type
      1,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

    { PC_HS1,                       // CID
      STR("PC_HS1"),                    // Param Name
      STR("on/off"),                    // Units
      MB_DEVICE_ADDR1,                  // Modbus Slave Addr
      MB_PARAM_COIL,                    // Modbus Reg Type
      0,                                // Reg Start
      1,                                // Reg Size
      0,                                // Instance Offset
      PARAM_TYPE_U8,                    // Data Type
      1,                                // Data Size
      OPTS( 0,0,0 ),                    // Parameter options MIN-MAX-STEP
      PAR_PERMS_READ_WRITE_TRIGGER      // Access Mode
    },

};

// Calculate number of parameters in the table
uint16_t num_PC_device_parameters = (sizeof(PC_device_parameters) / sizeof(PC_device_parameters[0]));