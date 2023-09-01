/*
    Macro definitions for each operation modes
    Each macro follows the notation:
        #define OPMode_Type_VarName ...  // Description
*/

#ifndef REMOTA_VAR_DEFS
#define REMOTA_VAR_DEFS

//Macros for: Natural flow wells
//_______________________________________________________________________________________________________________
#define NF_AI_PTL s3Tables.anTbl[0][0]      // Presión de la línea de producción
#define NF_AI_TTL s3Tables.anTbl[0][1]      // Temperatura de la línea de producción
#define NF_AI_PTC s3Tables.anTbl[0][2]      // Presión de cabezal
#define NF_AI_PTA s3Tables.anTbl[0][3]      // Presión del casing o anular
#define NF_AI_TTA s3Tables.anTbl[0][4]      // Temperatura del casing o anular
#define NF_AI_FT  s3Tables.anTbl[0][5]      // Flujo de producción

#define NF_DI_YA1   (s3Tables.digTbl[0][0] & 0x0001)        // Alarma por falla de sistema
#define NF_DI_YA2   (s3Tables.digTbl[0][0] & 0x0002)  >> 1  // Alarma de intruso
#define NF_DI_GZA   (s3Tables.digTbl[0][0] & 0x0004)  >> 2  // Alarma por gas tóxico
#define NF_DI_EAAC  (s3Tables.digTbl[0][0] & 0x0008)  >> 3  // Alarma por falla de voltaje AC
#define NF_DI_EADC  (s3Tables.digTbl[0][0] & 0x00016) >> 4  // Alarma por falla de voltaje DC

#define NF_DO_XV    (s3Tables.digTbl[1][0] & 0x0001)        // Apertura/cierre de pozo

#define NF_SF_PTL s3Tables.scalingFactor[0]      // Presión de la línea de producción (Factor de escalamiento)
#define NF_SF_TTL s3Tables.scalingFactor[1]      // Temperatura de la línea de producción (Factor de escalamiento)
#define NF_SF_PTC s3Tables.scalingFactor[2]      // Presión de cabezal (Factor de escalamiento)
#define NF_SF_PTA s3Tables.scalingFactor[3]      // Presión del casing o anular (Factor de escalamiento)
#define NF_SF_TTA s3Tables.scalingFactor[4]      // Temperatura del casing o anular (Factor de escalamiento)
#define NF_SF_FT  s3Tables.scalingFactor[5]      // Flujo de producción (Factor de escalamiento)

#define NF_SO_PTL s3Tables.scalingOffset[0]      // Presión de la línea de producción (Offset de escalamiento)
#define NF_SO_TTL s3Tables.scalingOffset[1]      // Temperatura de la línea de producción (Offset de escalamiento)
#define NF_SO_PTC s3Tables.scalingOffset[2]      // Presión de cabezal (Offset de escalamiento)
#define NF_SO_PTA s3Tables.scalingOffset[3]      // Presión del casing o anular (Offset de escalamiento)
#define NF_SO_TTA s3Tables.scalingOffset[4]      // Temperatura del casing o anular (Offset de escalamiento)
#define NF_SO_FT  s3Tables.scalingOffset[5]      // Flujo de producción (Offset de escalamiento)

#define NF_SV_PTL s3Tables.scaledValues[0]       // Presión de la línea de producción (Valor escalado)
#define NF_SV_TTL s3Tables.scaledValues[1]       // Temperatura de la línea de producción (Valor escalado)
#define NF_SV_PTC s3Tables.scaledValues[2]       // Presión de cabezal (Valor escalado)
#define NF_SV_PTA s3Tables.scaledValues[3]       // Presión del casing o anular (Valor escalado)
#define NF_SV_TTA s3Tables.scaledValues[4]       // Temperatura del casing o anular (Valor escalado)
#define NF_SV_FT  s3Tables.scaledValues[5]       // Flujo de producción (Valor escalado)

//Macros for: Gas lift wells
//_______________________________________________________________________________________________________________
#define GL_AI_PTL  s3Tables.anTbl[0][0]      // Presión de la línea de producción
#define GL_AI_TTL  s3Tables.anTbl[0][1]      // Temperatura de la línea de producción
#define GL_AI_ZT   s3Tables.anTbl[0][2]      // Posición válvula control flujo al pozo
#define GL_AI_PTGL s3Tables.anTbl[0][3]      // Presión de gas al pozo
#define GL_AI_TTGL s3Tables.anTbl[0][4]      // Temperatura de gas al pozo
#define GL_AI_FTGL s3Tables.anTbl[0][5]      // Flujo de gas al pozo

#define GL_AO_FCV  s3Tables.anTbl[1][0]      // Flujo de gas al pozo

#define GL_DI_YA1   (s3Tables.digTbl[0][0] & 0x0001)        // Alarma por falla de sistema
#define GL_DI_YA2   (s3Tables.digTbl[0][0] & 0x0002)  >> 1  // Alarma de intruso
#define GL_DI_GZA   (s3Tables.digTbl[0][0] & 0x0004)  >> 2  // Alarma por gas tóxico
#define GL_DI_EAAC  (s3Tables.digTbl[0][0] & 0x0008)  >> 3  // Alarma por falla de voltaje AC
#define GL_DI_EADC  (s3Tables.digTbl[0][0] & 0x00016) >> 4  // Alarma por falla de voltaje DC

#define GL_FQ      s3Tables.auxTbl[0][5]     // Volumen total diario de gas

#define GL_SF_PTL  s3Tables.scalingFactor[0]      // Presión de la línea de producción (Factor de escalamiento)
#define GL_SF_TTL  s3Tables.scalingFactor[1]      // Temperatura de la línea de producción (Factor de escalamiento)
#define GL_SF_ZT   s3Tables.scalingFactor[2]      // Posición válvula control flujo al pozo (Factor de escalamiento)
#define GL_SF_PTGL s3Tables.scalingFactor[3]      // Presión de gas al pozo (Factor de escalamiento)
#define GL_SF_TTGL s3Tables.scalingFactor[4]      // Temperatura de gas al pozo (Factor de escalamiento)
#define GL_SF_FTGL s3Tables.scalingFactor[5]      // Flujo de gas al pozo (Factor de escalamiento)

#define GL_SO_PTL  s3Tables.scalingOffset[0]      // Presión de la línea de producción (Offset de escalamiento)
#define GL_SO_TTL  s3Tables.scalingOffset[1]      // Temperatura de la línea de producción (Offset de escalamiento)
#define GL_SO_ZT   s3Tables.scalingOffset[2]      // Posición válvula control flujo al pozo (Offset de escalamiento)
#define GL_SO_PTGL s3Tables.scalingOffset[3]      // Presión de gas al pozo (Offset de escalamiento)
#define GL_SO_TTGL s3Tables.scalingOffset[4]      // Temperatura de gas al pozo (Offset de escalamiento)
#define GL_SO_FTGL s3Tables.scalingOffset[5]      // Flujo de gas al pozo (Offset de escalamiento)

#define GL_SV_PTL  s3Tables.scaledValues[0]       // Presión de la línea de producción (Valor escalado)
#define GL_SV_TTL  s3Tables.scaledValues[1]       // Temperatura de la línea de producción (Valor escalado)
#define GL_SV_ZT   s3Tables.scaledValues[2]       // Posición válvula control flujo al pozo (Valor escalado)
#define GL_SV_PTGL s3Tables.scaledValues[3]       // Presión de gas al pozo (Valor escalado)
#define GL_SV_TTGL s3Tables.scaledValues[4]       // Temperatura de gas al pozo (Valor escalado)
#define GL_SV_FTGL s3Tables.scaledValues[5]       // Flujo de gas al pozo (Valor escalado)

//Macros for: Mechanical pump wells
//_______________________________________________________________________________________________________________
#define MP_AI_PTL s3Tables.anTbl[0][0]      // Presión de la línea de producción
#define MP_AI_TTL s3Tables.anTbl[0][1]      // Temperatura de la línea de producción
#define MP_AI_PTA s3Tables.anTbl[0][2]      // Presión del casing o anular
#define MP_AI_PTC s3Tables.anTbl[0][3]      // Presión de cabezal
#define MP_AI_SPM s3Tables.anTbl[0][4]      // Golpe por minuto (spm)
#define MP_AI_ZT  s3Tables.anTbl[0][5]      // Ángulo de la viga
#define MP_AI_WT  s3Tables.anTbl[0][6]      // Carga de la sarta

#define MP_DI_YA1   (s3Tables.digTbl[0][0] & 0x0001)  // Alarma por falla de sistema
#define MP_DI_YA2   (s3Tables.digTbl[0][0] & 0x0002)  >> 1  // Alarma de intruso
#define MP_DI_GZA   (s3Tables.digTbl[0][0] & 0x0004)  >> 2  // Alarma por gas tóxico
#define MP_DI_EAAC  (s3Tables.digTbl[0][0] & 0x0008)  >> 3  // Alarma por falla de voltaje AC
#define MP_DI_EADC  (s3Tables.digTbl[0][0] & 0x00016) >> 4  // Alarma por falla de voltaje DC

#define MP_DO_HS1   (s3Tables.digTbl[1][0] & 0x0001)  // Encendido/apagado de bombeo

#define MP_SF_PTL  s3Tables.scalingFactor[0]      // Presión de la línea de producción (Factor de escalamiento)
#define MP_SF_TTL  s3Tables.scalingFactor[1]      // Temperatura de la línea de producción (Factor de escalamiento)
#define MP_SF_PTA  s3Tables.scalingFactor[2]      // Presión del casing o anular (Factor de escalamiento)
#define MP_SF_PTC  s3Tables.scalingFactor[3]      // Presión de cabezal (Factor de escalamiento)
#define MP_SF_SPM  s3Tables.scalingFactor[4]      // Golpe por minuto (spm) (Factor de escalamiento)
#define MP_SF_ZT   s3Tables.scalingFactor[5]      // Ángulo de la viga (Factor de escalamiento)
#define MP_SF_WT   s3Tables.scalingFactor[6]      // Carga de la sarta (Factor de escalamiento)

#define MP_SO_PTL  s3Tables.scalingOffset[0]      // Presión de la línea de producción (Offset de escalamiento)
#define MP_SO_TTL  s3Tables.scalingOffset[1]      // Temperatura de la línea de producción (Offset de escalamiento)
#define MP_SO_PTA  s3Tables.scalingOffset[2]      // Presión del casing o anular (Offset de escalamiento)
#define MP_SO_PTC  s3Tables.scalingOffset[3]      // Presión de cabezal (Offset de escalamiento)
#define MP_SO_SPM  s3Tables.scalingOffset[4]      // Golpe por minuto (spm) (Offset de escalamiento)
#define MP_SO_ZT   s3Tables.scalingOffset[5]      // Ángulo de la viga (Offset de escalamiento)
#define MP_SO_WT   s3Tables.scalingOffset[6]      // Carga de la sarta (Offset de escalamiento)

#define MP_SV_PTL  s3Tables.scaledValues[0]       // Presión de la línea de producción (Valor escalado)
#define MP_SV_TTL  s3Tables.scaledValues[1]       // Temperatura de la línea de producción (Valor escalado)
#define MP_SV_PTA  s3Tables.scaledValues[2]       // Presión del casing o anular (Valor escalado)
#define MP_SV_PTC  s3Tables.scaledValues[3]       // Presión de cabezal (Valor escalado)
#define MP_SV_SPM  s3Tables.scaledValues[4]       // Golpe por minuto (spm) (Valor escalado)
#define MP_SV_ZT   s3Tables.scaledValues[5]       // Ángulo de la viga (Valor escalado)
#define MP_SV_WT   s3Tables.scaledValues[6]       // Carga de la sarta (Valor escalado)

#define MP_IR_ETM  s3Tables.mbTblFloat[0][0]      // Voltaje de motor
#define MP_IR_ITM  s3Tables.mbTblFloat[0][1]      // Corriente de motor
#define MP_IR_JTM  s3Tables.mbTblFloat[0][2]      // Potencia de motor

#define MP_HR_SCM  s3Tables.mbTblFloat[0][3]      // Ajuste de velocidad

#define MP_COIL_HS1 (s3Tables.mbTbl8bit[0][0] & 0x01) // Encendido/apagado de bombeo

//Macros for: Electro-Submersible pump wells
//_______________________________________________________________________________________________________________
#define ES_AI_PTL   s3Tables.anTbl[0][0]      // Presión de la línea de producción
#define ES_AI_TTL   s3Tables.anTbl[0][1]      // Temperatura de la línea de producción
#define ES_AI_PTA   s3Tables.anTbl[0][2]      // Presión del casing o anular
#define ES_AI_PTC   s3Tables.anTbl[0][3]      // Presión de cabezal
#define ES_AI_PTSB  s3Tables.anTbl[0][4]      // Presión de succión de la bomba
#define ES_AI_PTDB  s3Tables.anTbl[0][5]      // Presión de descarga de la bomba
#define ES_AI_TTSB  s3Tables.anTbl[0][6]      // Temperatura de succión de la bomba
#define ES_AI_TTDB  s3Tables.anTbl[0][7]      // Temperatura de descarga de la bomba
#define ES_AI_VT    s3Tables.anTbl[0][8]      // Vibración de bomba

#define ES_DI_YA1   (s3Tables.digTbl[0][0] & 0x0001)        // Alarma por falla de sistema
#define ES_DI_YA2   (s3Tables.digTbl[0][0] & 0x0002) >> 1   // Alarma de intruso
#define ES_DI_GZA   (s3Tables.digTbl[0][0] & 0x0004) >> 2   // Alarma por gas tóxico
#define ES_DI_EAAC  (s3Tables.digTbl[0][0] & 0x0008) >> 3   // Alarma por falla de voltaje AC
#define ES_DI_EADC  (s3Tables.digTbl[0][0] & 0x00016) >> 4  // Alarma por falla de voltaje DC

#define ES_DO_HS1   (s3Tables.digTbl[1][0] & 0x0001)  // Encendido/apagado de bombeo

#define ES_SF_PTL   s3Tables.scalingFactor[0]      // Presión de la línea de producción (Factor de escalamiento)
#define ES_SF_TTL   s3Tables.scalingFactor[1]      // Temperatura de la línea de producción (Factor de escalamiento)
#define ES_SF_PTA   s3Tables.scalingFactor[2]      // Presión del casing o anular (Factor de escalamiento)
#define ES_SF_PTC   s3Tables.scalingFactor[3]      // Presión de cabezal (Factor de escalamiento)
#define ES_SF_PTSB  s3Tables.scalingFactor[4]      // Presión de succión de la bomba (Factor de escalamiento)
#define ES_SF_PTDB  s3Tables.scalingFactor[5]      // Presión de descarga de la bomba (Factor de escalamiento)
#define ES_SF_TTSB  s3Tables.scalingFactor[6]      // Temperatura de succión de la bomba (Factor de escalamiento)
#define ES_SF_TTDB  s3Tables.scalingFactor[7]      // Temperatura de descarga de la bomba (Factor de escalamiento)
#define ES_SF_VT    s3Tables.scalingFactor[8]      // Vibración de bomba (Factor de escalamiento)

#define ES_SO_PTL   s3Tables.scalingOffset[0]      // Presión de la línea de producción (Offset de escalamiento)
#define ES_SO_TTL   s3Tables.scalingOffset[1]      // Temperatura de la línea de producción (Offset de escalamiento)
#define ES_SO_PTA   s3Tables.scalingOffset[2]      // Presión del casing o anular (Offset de escalamiento)
#define ES_SO_PTC   s3Tables.scalingOffset[3]      // Presión de cabezal (Offset de escalamiento)
#define ES_SO_PTSB  s3Tables.scalingOffset[4]      // Presión de succión de la bomba (Offset de escalamiento)
#define ES_SO_PTDB  s3Tables.scalingOffset[5]      // Presión de descarga de la bomba (Offset de escalamiento)
#define ES_SO_TTSB  s3Tables.scalingOffset[6]      // Temperatura de succión de la bomba (Offset de escalamiento)
#define ES_SO_TTDB  s3Tables.scalingOffset[7]      // Temperatura de descarga de la bomba (Offset de escalamiento)
#define ES_SO_VT    s3Tables.scalingOffset[8]      // Vibración de bomba (Offset de escalamiento)

#define ES_SV_PTL   s3Tables.scaledValues[0]       // Presión de la línea de producción (Valor escalado)
#define ES_SV_TTL   s3Tables.scaledValues[1]       // Temperatura de la línea de producción (Valor escalado)
#define ES_SV_PTA   s3Tables.scaledValues[2]       // Presión del casing o anular (Valor escalado)
#define ES_SV_PTC   s3Tables.scaledValues[3]       // Presión de cabezal (Valor escalado)
#define ES_SV_PTSB  s3Tables.scaledValues[4]       // Presión de succión de la bomba (Valor escalado)
#define ES_SV_PTDB  s3Tables.scaledValues[5]       // Presión de descarga de la bomba (Valor escalado)
#define ES_SV_TTSB  s3Tables.scaledValues[6]       // Temperatura de succión de la bomba (Valor escalado)
#define ES_SV_TTDB  s3Tables.scaledValues[7]       // Temperatura de descarga de la bomba (Valor escalado)
#define ES_SV_VT    s3Tables.scaledValues[8]       // Vibración de bomba (Valor escalado)

#define ES_IR_ETM   s3Tables.mbTblFloat[0][0]       // Voltaje de motor
#define ES_IR_ITM   s3Tables.mbTblFloat[0][1]       // Corriente de motor
#define ES_IR_ST1   s3Tables.mbTblFloat[0][2]       // Frecuencia de salida
#define ES_IR_TTM   s3Tables.mbTblFloat[0][3]       // Temperatura de arrollado de motor
#define ES_IR_WTM   s3Tables.mbTblFloat[0][4]       // Torque de motor
#define ES_IR_WTB   s3Tables.mbTblFloat[0][5]       // Torque de bomba
#define ES_IR_ST2   s3Tables.mbTblFloat[0][6]       // Velocidad de la bomba
#define ES_IR_SFM   s3Tables.mbTblFloat[0][7]       // Relación de poleas en caja de velocidad (VFD)
#define ES_IR_PTSB  s3Tables.mbTblFloat[0][8]       // Presión de succión de la bomba
#define ES_IR_PTDB  s3Tables.mbTblFloat[0][9]       // Presión de descarga de la bomba
#define ES_IR_TTSB  s3Tables.mbTblFloat[0][10]      // Temperatura de succión de la bomba
#define ES_IR_TTDB  s3Tables.mbTblFloat[0][11]      // Temperatura de descarga de la bomba
#define ES_IR_VT    s3Tables.mbTblFloat[0][12]      // Vibración de bomba

#define ES_IR_YI1   s3Tables.mbTbl16bit[0][0]       // Variables de Estatus del variador (arrancado, parado, back spin, entre otros)
#define ES_IR_YI2   s3Tables.mbTbl16bit[0][1]       // Fallas actuales del VFD

#define ES_HR_SCM   s3Tables.mbTblFloat[0][13]      // Velocidad de la bomba

#define ES_DISCRETE_YI (s3Tables.mbTbl8bit[0][0] & 0x01)        // Encendido/apagado

#define ES_COIL_HS1    (s3Tables.mbTbl8bit[0][1] & 0x01)        // Encendido/apagado
#define ES_COIL_HS2    (s3Tables.mbTbl8bit[0][1] & 0x02) >> 1   // Reposición de causa de paro

//Macros for: Progressive cavity pump wells
//_______________________________________________________________________________________________________________
#define PC_AI_PTL   s3Tables.anTbl[0][0]      // Presión de la línea de producción
#define PC_AI_TTL   s3Tables.anTbl[0][1]      // Temperatura de la línea de producción
#define PC_AI_PTSB  s3Tables.anTbl[0][2]      // Presión de succión de la bomba
#define PC_AI_PTDB  s3Tables.anTbl[0][3]      // Presión de descarga de la bomba
#define PC_AI_TTSB  s3Tables.anTbl[0][4]      // Temperatura de succión de la bomba
#define PC_AI_TTDB  s3Tables.anTbl[0][5]      // Temperatura de descarga de la bomba
#define PC_AI_VT    s3Tables.anTbl[0][6]      // Vibración de bomba

#define PC_DI_YA1   (s3Tables.digTbl[0][0] & 0x0001)        // Alarma por falla de sistema
#define PC_DI_YA2   (s3Tables.digTbl[0][0] & 0x0002)  >> 1  // Alarma de intruso
#define PC_DI_GZA   (s3Tables.digTbl[0][0] & 0x0004)  >> 2  // Alarma por gas tóxico
#define PC_DI_EAAC  (s3Tables.digTbl[0][0] & 0x0008)  >> 3  // Alarma por falla de voltaje AC
#define PC_DI_EADC  (s3Tables.digTbl[0][0] & 0x00016) >> 4  // Alarma por falla de voltaje DC

#define PC_DO_HS1   (s3Tables.digTbl[1][0] & 0x0001)        // Encendido/apagado de bombeo

#define PC_SF_PTL   s3Tables.scalingFactor[0]      // Presión de la línea de producción (Factor de escalamiento)
#define PC_SF_TTL   s3Tables.scalingFactor[1]      // Temperatura de la línea de producción (Factor de escalamiento)
#define PC_SF_PTSB  s3Tables.scalingFactor[2]      // Presión de succión de la bomba (Factor de escalamiento)
#define PC_SF_PTDB  s3Tables.scalingFactor[3]      // Presión de descarga de la bomba (Factor de escalamiento)
#define PC_SF_TTSB  s3Tables.scalingFactor[4]      // Temperatura de succión de la bomba (Factor de escalamiento)
#define PC_SF_TTDB  s3Tables.scalingFactor[5]      // Temperatura de descarga de la bomba (Factor de escalamiento)
#define PC_SF_VT    s3Tables.scalingFactor[6]      // Vibración de bomba (Factor de escalamiento)

#define PC_SO_PTL   s3Tables.scalingOffset[0]      // Presión de la línea de producción (Offset de escalamiento)
#define PC_SO_TTL   s3Tables.scalingOffset[1]      // Temperatura de la línea de producción (Offset de escalamiento)
#define PC_SO_PTSB  s3Tables.scalingOffset[2]      // Presión de succión de la bomba (Offset de escalamiento)
#define PC_SO_PTDB  s3Tables.scalingOffset[3]      // Presión de descarga de la bomba (Offset de escalamiento)
#define PC_SO_TTSB  s3Tables.scalingOffset[4]      // Temperatura de succión de la bomba (Offset de escalamiento)
#define PC_SO_TTDB  s3Tables.scalingOffset[5]      // Temperatura de descarga de la bomba (Offset de escalamiento)
#define PC_SO_VT    s3Tables.scalingOffset[6]      // Vibración de bomba (Offset de escalamiento)

#define PC_SV_PTL   s3Tables.scaledValues[0]       // Presión de la línea de producción (Valor escalado)
#define PC_SV_TTL   s3Tables.scaledValues[1]       // Temperatura de la línea de producción (Valor escalado)
#define PC_SV_PTSB  s3Tables.scaledValues[2]       // Presión de succión de la bomba (Valor escalado)
#define PC_SV_PTDB  s3Tables.scaledValues[3]       // Presión de descarga de la bomba (Valor escalado)
#define PC_SV_TTSB  s3Tables.scaledValues[4]       // Temperatura de succión de la bomba (Valor escalado)
#define PC_SV_TTDB  s3Tables.scaledValues[5]       // Temperatura de descarga de la bomba (Valor escalado)
#define PC_SV_VT    s3Tables.scaledValues[6]       // Vibración de bomba (Valor escalado)

#define PC_IR_WTM   s3Tables.mbTblFloat[0][0]       // Torque de motor
#define PC_IR_WTB   s3Tables.mbTblFloat[0][1]       // Torque de bomba
#define PC_IR_ST2   s3Tables.mbTblFloat[0][2]       // Velocidad de la bomba
#define PC_IR_SFM   s3Tables.mbTblFloat[0][3]       // Relación de poleas en caja de velocidad (VFD)
#define PC_IR_PTSB  s3Tables.mbTblFloat[0][4]       // Presión de succión de la bomba
#define PC_IR_PTDB  s3Tables.mbTblFloat[0][5]       // Presión de descarga de la bomba
#define PC_IR_TTSB  s3Tables.mbTblFloat[0][6]       // Temperatura de succión de la bomba
#define PC_IR_TTDB  s3Tables.mbTblFloat[0][7]       // Temperatura de descarga de la bomba
#define PC_IR_VT    s3Tables.mbTblFloat[0][8]       // Vibración de bomba

#define PC_IR_YI1   s3Tables.mbTbl16bit[0][0]       // Variables de Estatus del variador (arrancado, parado, back spin, entre otros)
#define PC_IR_YI2   s3Tables.mbTbl16bit[0][1]       // Fallas actuales del VFD

#define PC_HR_SCM   s3Tables.mbTblFloat[0][9]      // Velocidad de la bomba

#define PC_DISCRETE_YI (s3Tables.mbTbl8bit[0][0] & 0x01)      // Encendido/apagado

#define PC_COIL_HS1    (s3Tables.mbTbl8bit[0][1] & 0x01)      // Encendido/apagado

//Macros for: Valve station
//_______________________________________________________________________________________________________________
#define VS_AI_PT   s3Tables.anTbl[0][0]      // Transmisor de presión
#define VS_AI_TT   s3Tables.anTbl[0][1]      // Transmisor de temperatura
#define VS_AI_AT   s3Tables.anTbl[0][2]      // Transmisor de interface (CONTROLOTRÓN)

#define VS_DI_YA1   (s3Tables.digTbl[0][0] & 0x0001)        // Alarma por falla de sistema
#define VS_DI_YA2   (s3Tables.digTbl[0][0] & 0x0002) >> 1   // Alarma de intruso
#define VS_DI_EAAC  (s3Tables.digTbl[0][0] & 0x0004) >> 2   // Alarma por falla de voltaje AC
#define VS_DI_EADC  (s3Tables.digTbl[0][0] & 0x0008) >> 3   // Alarma por falla de voltaje DC

#define VS_DI_XI   (s3Tables.digTbl[0][1] & 0x0001)         // Detector de herramienta de limpieza (Raspatubo)
#define VS_DI_LNL  (s3Tables.digTbl[0][1] & 0x0002) >> 1    // Bajo nivel de nitrógeno
#define VS_DI_OV   (s3Tables.digTbl[0][1] & 0x0004) >> 2    // Válvula completamente abierta
#define VS_DI_CV   (s3Tables.digTbl[0][1] & 0x0008) >> 3    // Válvula completamente cerrada

#define VS_DO_XV   (s3Tables.digTbl[1][0] & 0x0001)         // Cierre rápido de la válvula

#define VS_SF_PT   s3Tables.scalingFactor[0]      // Transmisor de presión (Factor de escalamiento)
#define VS_SF_TT   s3Tables.scalingFactor[1]      // Transmisor de temperatura (Factor de escalamiento)
#define VS_SF_AT   s3Tables.scalingFactor[2]      // Transmisor de interface (CONTROLOTRÓN) (Factor de escalamiento)

#define VS_SO_PT   s3Tables.scalingOffset[0]      // Transmisor de presión (Offset de escalamiento)
#define VS_SO_TT   s3Tables.scalingOffset[1]      // Transmisor de temperatura (Offset de escalamiento)
#define VS_SO_AT   s3Tables.scalingOffset[2]      // Transmisor de interface (CONTROLOTRÓN) (Offset de escalamiento)

#define VS_SV_PT   s3Tables.scaledValues[0]       // Transmisor de presión (Valor escalado)
#define VS_SV_TT   s3Tables.scaledValues[1]       // Transmisor de temperatura (Valor escalado)
#define VS_SV_AT   s3Tables.scaledValues[2]       // Transmisor de interface (CONTROLOTRÓN) (Valor escalado)

#endif