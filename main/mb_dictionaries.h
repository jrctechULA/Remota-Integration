//#include "mbcontroller.h"

// Enumeration of modbus slave addresses accessed by master device
enum {
    MB_DEVICE_ADDR1 = 1,
    MB_SLAVE_COUNT
};


//____________________________________________________________________________________________________
// Mechanical Punp Wells Slave Definitions:
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
