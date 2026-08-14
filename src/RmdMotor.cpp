#include "RmdMotor.h"

//Commands
#define SET_TORQUE_COMMAND          0xA1
#define REQUEST_POS_COMMAND         0x92
#define SET_ZERO_POS_COMMAND_V1     0x19 // Only for RMD V1-V2
#define SET_ZERO_POS_COMMAND_V3     0x64 // Only for RMD V3
#define TURN_OFF_COMMAND            0X80
#define UPDATE_STATUS_COMMAND       0x9C

// New Commads
#define REQUEST_PID_COMMAND           0x30
#define SET_PID_COMMAND               0x31
#define REQUEST_ACCELERATION_COMMAND  0x42
#define SET_ACCELERATION_COMMAND      0x43
#define SET_POSITION_COMMAND          0xA4

//Motors Parameters
#define REDUCTION_1_TO_1      1.00f
#define REDUCTION_6_TO_1      6.00f
#define REDUCTION_6_2_TO_1    6.20f
#define REDUCTION_9_TO_1      9.00f
#define L5015_KT              0.08f
#define X6_KT                 0.88f
#define X8_PRO_V1_KT          3.30f
#define X8_V1_KT              3.00f
#define X8_PRO_V2_KT          2.60f
#define X8_V2_KT              2.09f
#define X8_PRO_V3_KT          0.29f
#define X8_V3_KT              0.30f

#define L7025_V2_KT           0.88f

#define CURRENT_RAW_MIN       -2000
#define CURRENT_RAW_MAX       2000
//Conversion Constants
#define AMPS_TO_RAW           62.5f           //Constant to convert from Amperes to RAW (in the motor manufacturer range).
#define RAW_TO_AMPS           0.016f           //Constant to convert from RAW to Amperes (in the motor manufacturer range).
#define RAD                   0.01745329251f   //(pi/180)grad to rad 
#define POSITIVE              1.0f
#define NEGATIVE             -1.0f
#define POS_CONVER_V3         0.9677419662F // Position value correction for RMD V3 motors (6.0F/6.2F)

//Definition of static constants. 
const RmdMotor::MotorType RmdMotor::RMD_X6{REDUCTION_6_TO_1, X6_KT, POSITIVE};                  
const RmdMotor::MotorType RmdMotor::RMD_X8_V1{REDUCTION_6_TO_1, X8_V1_KT, NEGATIVE};
const RmdMotor::MotorType RmdMotor::RMD_X8_PRO_V1{REDUCTION_6_TO_1, X8_PRO_V1_KT, NEGATIVE};
//const RmdMotor::MotorType RmdMotor::RMD_X8_V2{REDUCTION_9_TO_1, X8_V2_KT, POSITIVE};
//const RmdMotor::MotorType RmdMotor::RMD_X8_PRO_V2{REDUCTION_9_TO_1, X8_PRO_V2_KT, POSITIVE};
const RmdMotor::MotorType RmdMotor::RMD_X8_V3{REDUCTION_1_TO_1, X8_V3_KT, NEGATIVE}; 
const RmdMotor::MotorType RmdMotor::RMD_X8_PRO_V3{REDUCTION_1_TO_1, X8_PRO_V3_KT, POSITIVE};      //ToDo: Verify direction sign.
const RmdMotor::MotorType RmdMotor::RMD_L5015{REDUCTION_1_TO_1, L5015_KT, NEGATIVE};
const RmdMotor::MotorType RmdMotor::RMD_L7025{REDUCTION_1_TO_1, L7025_V2_KT, NEGATIVE};



RmdMotor::RmdMotor(const MotorType & motor_type, const uint8_t _CS, const uint8_t _INT_PIN, const char * motor_name, SPIClass & spi, const bool doBegin)
    : CanMotor{_CS, _INT_PIN, motor_name, spi, doBegin}, m_motor_type(motor_type)
{
}



bool RmdMotor::turnOn()
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = UPDATE_STATUS_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;

    can_msg.data[0] = REQUEST_POS_COMMAND;
    return m_sendAndReceiveBlocking(can_msg, 1000000);
}



bool RmdMotor::turnOff() 
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = TURN_OFF_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;

    can_msg.data[0] = REQUEST_POS_COMMAND;
    return m_sendAndReceiveBlocking(can_msg, 1000000);
}



//Refactor this function.
bool RmdMotor::setTorque(float torque_setpoint )
{
    //Normal mode (Polling based)
    if(!m_is_auto_mode_running) return m_sendTorque(torque_setpoint);


    //Auto mode (Interrupt driven)
    if (m_is_response_ready)
    {
        m_is_response_ready = false;
        uint8_t irq = m_mcp2515.getInterrupts();
        if (irq & MCP2515::CANINTF_MERRF)
        {
            Serial.print("\n\n!!!!!ERROR MERF (ERROR IN MESSAGE TRANSMISSION OR RECEPTION)!!!"); Serial.print(m_name); Serial.print("\n\n");
            m_mcp2515.clearMERR();
            m_mcp2515.clearInterrupts();
        }
        if (irq & MCP2515::CANINTF_ERRIF)
        {
            Serial.print("\n\n!!!!!!!ERROR BUFFER FULL!!!!!!"); Serial.print(m_name); Serial.print("\n\n");
            m_mcp2515.clearRXnOVRFlags();
            m_mcp2515.clearERRIF();
            m_mcp2515.clearInterrupts();
        }
        m_readMotorResponse();
        if (m_curr_state == 0)
        {
            m_curr_state = 1;
            return m_sendTorque(torque_setpoint);
        }
        else
        {
            m_curr_state = 0;
            return m_requestPosition();
        }
    }
    else if ((millis() - m_last_response_time_ms) < MILLIS_LIMIT_UNTIL_RETRY) //In auto mode, test if we received response 
    {
        //Serial.println("All Ok, auto mode running normally");
        return true; //Everything OK. Motor doesn't need communication "recovery"
    }
    else
    {
        if ((millis() - m_last_retry_time_ms) > MILLIS_LIMIT_UNTIL_RETRY)
        {
            m_last_retry_time_ms = millis();
            //Serial.print("\t Millis: "); Serial.print(millis()); Serial.print("\tLast message"); Serial.print(m_last_response_time_ms); Serial.print("\tRetrying to recover "); Serial.println(m_name); 
            m_emptyMCP2515buffer();
            if (m_curr_state == 0)
            {
                m_curr_state = 1;
                m_sendTorque(torque_setpoint);
            }
            else
            {
                m_curr_state = 0;
                m_requestPosition();
            }
        }
        return false;
    }
}



bool RmdMotor::setTorque(float torque_setpoint, unsigned long timeout_us){
    if (m_is_auto_mode_running) 
    {
        return setTorque(torque_setpoint);
    }
    bool was_message_sent;
    unsigned long t_ini = micros();
    while(!(was_message_sent = setTorque(torque_setpoint)) and (micros()-t_ini) < timeout_us)
    {
        //Serial.println("Send Retry!");           
    }
    return was_message_sent;
}



bool RmdMotor::setCurrentPositionAsZero()
{
    stopAutoMode();
    can_frame can_msg;
    int command; 
    if (m_motor_type.KT == X8_V3_KT || m_motor_type.KT == X8_PRO_V3_KT)
    {
        command = SET_ZERO_POS_COMMAND_V3;
    }
    else
    {
        command = SET_ZERO_POS_COMMAND_V1;
    }
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = command;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;
    can_msg.data[0] = REQUEST_POS_COMMAND;
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;
    m_offset_from_zero_motor = m_position;
    return true;
}



bool RmdMotor::setCurrentPositionAsOrigin(){
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = REQUEST_POS_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    //actualizar m_offset... con el nuevo valor de posición. 
    if (! m_sendAndReceiveBlocking(can_msg, 1000000))
    {
        return false;
    }
    m_offset_from_zero_motor = m_position;
    return true;
}



bool RmdMotor::requestPosition()
{
    return (m_is_auto_mode_running ? true : m_requestPosition());
}

bool RmdMotor::requestPosition(unsigned long timeout_us)
{
    bool was_message_sent;
    unsigned long t_ini = micros();
    while(!(was_message_sent = requestPosition()) and (micros()-t_ini) < timeout_us)
    {
        //Serial.println("Send Retry!");           
    }
    return was_message_sent;
}


bool RmdMotor::m_sendTorque(float torque_setpoint)
{
    can_frame can_msg;
    // int16_t torque = constrain(((int16_t)(((torque_setpoint)/m_motor_type.KT)*AMPS_TO_RAW)), CURRENT_RAW_MIN, CURRENT_RAW_MAX) * m_motor_type.DIRECTION_SIGN;
    float contrainTorque = constrain(torque_setpoint * m_motor_type.DIRECTION_SIGN, -5.0, 5.0);
    int16_t torque = static_cast<int16_t>((contrainTorque * m_motor_type.KT) * 100);
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = SET_TORQUE_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = torque;
    can_msg.data[5] = torque >> 8;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    //m_mcp2515.clearInterrupts();
    bool result = (m_mcp2515.sendMessage(&can_msg) == MCP2515::ERROR_OK);
    return result;
}



bool RmdMotor::m_readMotorResponse()
{
    if(m_mcp2515.readMessage(&response_msg) != MCP2515::ERROR::ERROR_OK)
    { 
        return false;
    }
    /// unpack ints from can buffer ///
    switch (response_msg.data[0])
    {
        case SET_TORQUE_COMMAND:
        case UPDATE_STATUS_COMMAND:
        case SET_POSITION_COMMAND:
            m_temperature = response_msg.data[1];
            // m_torque = (int16_t(response_msg.data[3] << 8) | int16_t(response_msg.data[2])) * m_motor_type.KT  * RAW_TO_AMPS * m_motor_type.DIRECTION_SIGN;
            current = (response_msg.data[3] << 8) | response_msg.data[2];
            m_torque = (current / 100.0) * m_motor_type.KT  * m_motor_type.DIRECTION_SIGN;
            m_velocity = (int16_t(response_msg.data[5] << 8) | int16_t(response_msg.data[4])) * RAD / m_motor_type.reduction * m_motor_type.DIRECTION_SIGN;
            break;

        case REQUEST_POS_COMMAND:
            if (m_motor_type.KT == X8_V3_KT || m_motor_type.KT == X8_PRO_V3_KT)
            {
                m_position = ((((response_msg.data[7] << 24) | (response_msg.data[6] << 16) | (response_msg.data[5] << 8) | (response_msg.data[4]))/(m_motor_type.reduction * 100.0f))* RAD) * m_motor_type.DIRECTION_SIGN * POS_CONVER_V3; //! v3
            }
            else
            {
                m_position = ((((response_msg.data[4] << 24) | (response_msg.data[3] << 16) | (response_msg.data[2] << 8) | (response_msg.data[1]))/(m_motor_type.reduction * 100.0f))* RAD) * m_motor_type.DIRECTION_SIGN; //! rmd v1, v2
            }
            break;
            
        case SET_ZERO_POS_COMMAND_V1:
        case SET_ZERO_POS_COMMAND_V3:
            Serial.print("Recibida confirmación de seteo de cero. Motor "); Serial.println(m_name); Serial.println("!!!!NECESARIO REINICIAR ALIMENTACIÓN DEL MOTOR PARA QUE SEA VALIDO EL NUEVO CERO Y NO EXPLOTE ALV!!!");
            break;

        case TURN_OFF_COMMAND:
            Serial.print("Recibida confirmación de apagado. Motor: "); Serial.println(m_name);
            break;
        
        case SET_PID_COMMAND:
            Serial.print("Recibida confirmación de seteo de PID. Motor: "); Serial.println(m_name);
            break;

        case REQUEST_PID_COMMAND:
            Serial.print("Recibida respuesta de PID. Motor: "); Serial.println(m_name);
            Serial.print("Current P: "); Serial.println(response_msg.data[2]);
            Serial.print("Current I: "); Serial.println(response_msg.data[3]);
            Serial.print("Speed P: ");   Serial.println(response_msg.data[4]);
            Serial.print("Speed I: ");    Serial.println(response_msg.data[5]);
            Serial.print("Position P: "); Serial.println(response_msg.data[6]);
            Serial.print("Position I: "); Serial.println(response_msg.data[7]);
            break;

        case SET_ACCELERATION_COMMAND:
            Serial.print("Recibida confirmación de seteo de aceleración. Motor: "); Serial.println(m_name);
            break;

        case REQUEST_ACCELERATION_COMMAND:
            Serial.print("Recibida respuesta de aceleración. Motor: "); Serial.println(m_name);
            Serial.print("Aceleración: "); Serial.println((int32_t)((response_msg.data[7]<<24)|| (response_msg.data[6]<<16)|| (response_msg.data[5]<<8)|| (response_msg.data[4])));
            break;
        
        default:
            Serial.print("Se recibió una respuesta, pero no se reconoció el comando. Motor: "); Serial.println(m_name);
            return false;
            break;
    }
    return true;   
}


bool RmdMotor::m_requestPosition()
{
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = REQUEST_POS_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;
    //m_mcp2515.clearInterrupts();
    bool result = m_mcp2515.sendMessage(&can_msg) == MCP2515::ERROR_OK;
    return result;
}


/*
units pos: radians 
units speed: rad/s
*/
bool RmdMotor::setPosition(int32_t position_setpoint, uint16_t speed_setpoint)
{
    int32_t m_pos = position_setpoint * m_motor_type.DIRECTION_SIGN * m_motor_type.reduction * (1.0f / RAD) * 100.0f; //Convert to motor units
    uint16_t m_speed = speed_setpoint * (1.0f / RAD); //Convert to motor units
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = SET_POSITION_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = (uint8_t)(m_speed);
    can_msg.data[3] = (uint8_t)(m_speed >> 8);
    can_msg.data[4] = (uint8_t)(m_pos);
    can_msg.data[5] = (uint8_t)(m_pos >> 8);
    can_msg.data[6] = (uint8_t)(m_pos >> 16);
    can_msg.data[7] = (uint8_t)(m_pos >> 24);
    
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;

    can_msg.data[0] = REQUEST_POS_COMMAND;
    return m_sendAndReceiveBlocking(can_msg, 1000000);
}

/*
Range:100-60000
Units: 0.1 dps/s
Funcion index: 0-3 (18 pag)(2.5.4)(https://teika-machine.co.jp/wp-content/uploads/2024/10/RMD-X-Motor-Motion-Protocol-V3.91.pdf)
 */
bool RmdMotor::setAcceleration(uint32_t acceleration_setpoint, uint8_t funcion_index)
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = SET_ACCELERATION_COMMAND;
    can_msg.data[1] = funcion_index;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = (uint8_t)(acceleration_setpoint);
    can_msg.data[5] = (uint8_t)(acceleration_setpoint >> 8);
    can_msg.data[6] = (uint8_t)(acceleration_setpoint >> 16);
    can_msg.data[7] = (uint8_t)(acceleration_setpoint >> 24);
    
    if (!m_sendAndReceiveBlocking(can_msg, 1000000)) return false;
    
    can_msg.data[0] = REQUEST_POS_COMMAND;
    return m_sendAndReceiveBlocking(can_msg, 1000000);
}

/*
0-255
PID's Limits are hardcoded in the motor firmware. The user can set values from 0 to 255, but the motor will limit them to the range it has been programmed with.
*/
bool RmdMotor::setPID(uint8_t current_P, uint8_t current_I, uint8_t speed_P, uint8_t speed_I, uint8_t position_P, uint8_t position_I)
{
    stopAutoMode();
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = SET_PID_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = current_P;
    can_msg.data[3] = current_I;
    can_msg.data[4] = speed_P;
    can_msg.data[5] = speed_I;
    can_msg.data[6] = position_P;
    can_msg.data[7] = position_I;   
    
    if (!m_sendAndReceiveBlocking(can_msg, 1000000))
    {
        return false;
    }
    return true;
}

bool RmdMotor::requestPID()
{
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = REQUEST_PID_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;

    if (! m_sendAndReceiveBlocking(can_msg, 1000000))
    {
        return false;
    }
    return true;
}

bool RmdMotor::requestAcceleration()
{
    can_frame can_msg;
    can_msg.can_id  = 0x141;
    can_msg.can_dlc = 0x08;
    can_msg.data[0] = REQUEST_ACCELERATION_COMMAND;
    can_msg.data[1] = 0x00;
    can_msg.data[2] = 0x00;
    can_msg.data[3] = 0x00;
    can_msg.data[4] = 0x00;
    can_msg.data[5] = 0x00;
    can_msg.data[6] = 0x00;
    can_msg.data[7] = 0x00;

    if (! m_sendAndReceiveBlocking(can_msg, 1000000))
    {
        return false;
    }
    return true;
}