
// /*
//  * @file main.cpp
//  * @brief Velocity control example for AK10 motors using MIT and RMD protocols.
//  * 
//  * This application implements a Finite State Machine (FSM) to interact with 
//  * CAN-based motors. It provides a serial menu for initialization, activation,
//  * and a controlled velocity test loop.
//  */
// #include <Arduino.h>
// #include "MitMotor.h"
// #include "RmdMotor.h"

// // --- Configuration Constants ---
// #define KD  5.0             ///< Derivative gain for the PID controller (constant)
// #define VELOCITY_MOTOR 5.0f ///< Target velocity in rad/s

// // --- Pinout Definitions ---
// #define CS_0 22             ///< Chip Select pin for the MCP2515 CAN controller
// #define INT_0 15            ///< Interrupt pin for the MCP2515 CAN controller
// #define CTRL_PIN 4          ///< Control/Emergency stop pin (Active Low)

// /**
//  * @brief Array of motor objects.
//  * Currently configured with one AK10 motor using the MIT protocol.
//  */
// RmdMotor * motors[] = 
// {
//     new RmdMotor(RmdMotor::RMD_L7025, CS_0, INT_0, "RMD_0")
// };

// constexpr size_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

// /**
//  * @brief Array of interrupt service routines (ISR) for each motor.
//  * Each lambda function maps to the specific motor's interrupt handler.
//  */
// void(*interrupt_handlers[NUM_MOTORS])() = 
// {
//     [](){motors[0]->handleInterrupt();}
// };


// // --- Timing and Control Variables ---
// const long duration = 30;           ///< Test duration in seconds
// const long frecuency = 100;         ///< Control loop frequency in Hz
// unsigned long currentMillis;        ///< Current system time
// unsigned long previousMillisDuration; ///< Timestamp for test duration tracking
// unsigned long previousMillisFrecuency; ///< Timestamp for control frequency tracking
// bool flag = false;                  ///< Flag to signal completion of the test duration

// /**
//  * @brief States for the main Finite State Machine.
//  */
// enum State 
// {
//   STATE_IDLE,             ///< Displays the menu and waits for a transition
//   STATE_WAITING_INPUT,    ///< Waits for user input via Serial
//   STATE_PROCESSING_INPUT  ///< Executes the command associated with the user input
// };

// /**
//  * @brief Menu options for user interaction.
//  */
// enum OPTION : char
// {
//   OPTION_TURN_ON = '0',
//   OPTION_TURN_OFF = '1',
//   OPTION_READ_POSITION = '2',
//   OPTION_SET_POSITION = '3',
//   OPTION_READ_PID = '4',
//   OPTION_PID_PID = '5',
//   OPTION_READ_ACCELERATION = '6',
//   OPTION_SET_ACCELERATION = '7'
// };

// State currentState = STATE_IDLE; ///< Current FSM state

// void setup()
// {
//   delay(5000); // Wait for serial monitor to open
//   Serial.begin(115200);
//   SPI.begin();
  
//   pinMode(CTRL_PIN, INPUT_PULLUP);

//   // Initialize all registered motors
//   for (uint8_t i = 0; i < NUM_MOTORS; i++)
//   {
//     if (!motors[i]->initialize())
//     {
//       Serial.printf("ERROR: MCP INITIALIZE FAILED %s\n", motors[i]->name());
//     }
//     else
//     {
//       Serial.printf("MCP INITIALIZE SUCCEEDED %s\n", motors[i]->name());
//     }
//     delay(100);
//   }
// }

// void loop()
// { 
//   // Main FSM execution
//   switch (currentState) 
//   {
//     case STATE_IDLE:
//       // Print the UI menu
//       Serial.println("***--- MENU INBIODROID ---***");
//       Serial.print("Turn On: "); Serial.println(OPTION_TURN_ON - '0');
//       Serial.print("Turn Off: "); Serial.println(OPTION_TURN_OFF - '0');
//       Serial.print("Read Position: "); Serial.println(OPTION_READ_POSITION - '0');
//       Serial.print("Set Position: "); Serial.println(OPTION_SET_POSITION - '0');
//       Serial.print("Read PID: "); Serial.println(OPTION_READ_PID - '0');
//       Serial.print("Set PID: "); Serial.println(OPTION_PID_PID - '0');
//       Serial.print("Read Acceleration: "); Serial.println(OPTION_READ_ACCELERATION - '0');
//       Serial.print("Set Acceleration: "); Serial.println(OPTION_SET_ACCELERATION - '0');
//       Serial.println();
//       currentState = STATE_WAITING_INPUT;
//       break;

//     case STATE_WAITING_INPUT:
//       // Check for incoming serial commands
//       if (Serial.available() > 0) 
//       {
//         currentState = STATE_PROCESSING_INPUT;
//       }
//       break;

//     case STATE_PROCESSING_INPUT:
//     {
//       OPTION input = (OPTION)Serial.read();

//       // OPTION 0: Enable Motors
//       if (input == OPTION_TURN_ON)
//       {
//         for (uint8_t i = 0; i < NUM_MOTORS; i++)
//         {
//           if (!motors[i]->turnOn())
//           {
//             currentState = STATE_IDLE;
//             Serial.printf("TURN ON FAILED\n");
//             return;
//           }
//         }
//         Serial.printf("TURN ON SUCCEEDED\n");
//       }

//       // OPTION 1: Disable Motors
//       else if (input == OPTION_TURN_OFF)
//       {
//         for (uint8_t i = 0; i < NUM_MOTORS; i++)
//         {
//           if (!motors[i]->turnOff())
//           {
//             currentState = STATE_IDLE;
//             return;
//           }
//         }
//         Serial.printf("TURN OFF SUCCEEDED\n");
//       }

//       // OPTION 2: Read Position
//       else if (input == OPTION_READ_POSITION)
//       {
//         for (uint8_t i = 0; i < NUM_MOTORS; i++)
//         {
//           if (!motors[i]->requestPosition())
//           {
//             currentState = STATE_IDLE;
//             Serial.printf("READ POSITION FAILED\n");
//             return;
//           }
//           Serial.printf("Motor %s Position: %.2f rad\n", motors[i]->name(), motors[i]->position());
//         }
//       }

//       // OPTION 3: Set Position
//       else if (input == OPTION_SET_POSITION)
//       {
//         int32_t position_setpoint = 1000; // Example position setpoint
//         uint16_t speed_setpoint = 500;    // Example speed setpoint
//         for (uint8_t i = 0; i < NUM_MOTORS; i++)
//         {
//           if (!motors[i]->setPosition(position_setpoint, speed_setpoint))
//           {
//             currentState = STATE_IDLE;
//             Serial.printf("SET POSITION FAILED\n");
//             return;
//           }
//         }
//       }

//       // OPTION 4: Read PID
//       else if (input == OPTION_READ_PID)
//       {
//         for (uint8_t i = 0; i < NUM_MOTORS; i++)
//         {
//           if (!motors[i]->requestPID())
//           {
//             currentState = STATE_IDLE;
//             Serial.printf("READ PID FAILED\n");
//             return;
//           }
//         }
//       }

//       // OPTION 5: Set PID
//       else if (input == OPTION_PID_PID)
//       {
//         uint8_t current_P = 10, current_I = 5, speed_P = 20, speed_I = 10, position_P = 30, position_I = 15; // Example PID values
//         for (uint8_t i = 0; i < NUM_MOTORS; i++)
//         {
//           if (!motors[i]->setPID(current_P, current_I, speed_P, speed_I, position_P, position_I))
//           {
//             currentState = STATE_IDLE;
//             Serial.printf("SET PID FAILED\n");
//             return;
//           }
//         }
//       }

//       // OPTION 6: Read Acceleration
//       else if (input == OPTION_READ_ACCELERATION)
//       {
//         for (uint8_t i = 0; i < NUM_MOTORS; i++)
//         {
//           if (!motors[i]->requestAcceleration())
//           {
//             currentState = STATE_IDLE;
//             Serial.printf("READ ACCELERATION FAILED\n");
//             return;
//           }
//         }
//       }

//       // OPTION 7: Set Acceleration
//       else if (input == OPTION_SET_ACCELERATION)
//       {
//         uint32_t acceleration_setpoint = 1000; // Example acceleration setpoint
//         uint8_t function_index = 0;             // Example function index
//         for (uint8_t i = 0; i < NUM_MOTORS; i++)
//         {
//           if (!motors[i]->setAcceleration(acceleration_setpoint, function_index))
//           {
//             currentState = STATE_IDLE;
//             Serial.printf("SET ACCELERATION FAILED\n");
//             return;
//           }
//         }
//       }

//       else
//       {
//         Serial.println("ERROR: INVALID OPCION");
//       }
      
//       currentState = STATE_IDLE;
//       break;
//     }
//   }
// }

#include <Arduino.h>

void setup(){}

void loop(){}