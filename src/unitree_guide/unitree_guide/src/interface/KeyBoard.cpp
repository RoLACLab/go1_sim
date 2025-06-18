/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <fstream>
#include <string>
#include <algorithm>
#include <chrono>
#include <deque>
#include <sstream>

KeyBoard::KeyBoard() {
    userCmd = UserCommand::NONE;
    userValue.setZero();
    startTime = std::chrono::steady_clock::now();

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard() {
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
}

UserCommand KeyBoard::checkCmd() {
    switch (_c) {
    case '1': return UserCommand::L2_B;
    case '2': return UserCommand::L2_A;
    case '3': return UserCommand::L2_X;
    case '4': return UserCommand::START;
#ifdef COMPILE_WITH_MOVE_BASE
    case '5': return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
    case '0': return UserCommand::L1_X;
    case '9': return UserCommand::L1_A;
    case '8': return UserCommand::L1_Y;
    case ' ': userValue.setZero(); return UserCommand::NONE;
    default:  return UserCommand::NONE;
    }
}

void KeyBoard::changeValue() {
    switch (_c) {
    // Movement commands - fixed values
    case 'w': case 'W': userValue.ly = 0.15f; break;   // Forward
    case 's': case 'S': userValue.ly = -0.15f; break;  // Backward
    case 'd': case 'D': userValue.lx = 0.15f; break;   // Right
    case 'a': case 'A': userValue.lx = -0.15f; break;  // Left
    
    // Rotation commands - fixed values
    case 'i': case 'I': userValue.ry = 0.15f; break;   // Rotate up
    case 'k': case 'K': userValue.ry = -0.15f; break;  // Rotate down
    case 'l': case 'L': userValue.rx = 0.15f; break;   // Rotate right
    case 'j': case 'J': userValue.rx = -0.15f; break;  // Rotate left
    
    // Stop command - set all movement values to 0 (using z/Z)
    case 'z': case 'Z':
        userValue.ly = 0.0f;
        userValue.lx = 0.0f;
        userValue.ry = 0.0f;
        userValue.rx = 0.0f;
        std::cout << "Full stop (ZERO) - all movement values set to 0" << std::endl;
        break;
        
    default: break;
    }
}

void* KeyBoard::runKeyBoard(void *arg) {
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg) {
    std::cout << "KeyBoard thread started" << std::endl;
    
    // Initialization sequence
    std::cout << "Executing initialization sequence..." << std::endl;
    
    // Stand command
    _c = '2';
    userCmd = checkCmd();
    std::cout << "Stand command sent" << std::endl;
    _c = '\0';
    usleep(3000000); // 3 second delay to ensure stand completes
    
    // Walk mode command
    _c = '4';
    userCmd = checkCmd();
    std::cout << "Walk mode command sent" << std::endl;
    _c = '\0';
    usleep(4000000); // 4 second delay
    
    std::cout << "Initialization complete, starting file monitoring" << std::endl;
    
    // File monitoring loop
    while (true) {
        std::ifstream file("action_plan.txt");
        if (!file.is_open()) {
            file.open("/home/shivayogiakki/action_plan.txt");
        }
        
        if (file.is_open()) {
            std::string line;
            if (std::getline(file, line)) {
                std::istringstream iss(line);
                std::string cmdStr;
                float timeVal;
                
                if (iss >> cmdStr >> timeVal) {
                    if (!cmdStr.empty()) {
                        char command = cmdStr[0];
                        std::cout << "Executing command: '" << command << "'" << std::endl;
                        _c = command;
                        userCmd = checkCmd();
                        if (userCmd == UserCommand::NONE) {
                            changeValue();
                        }
                        _c = '\0';
                    }
                }
            }
            file.close();
        } else {
            std::cerr << "Failed to open action_plan.txt" << std::endl;
        }
        
        usleep(100000); // 100ms delay between checks
    }

    return NULL;
}