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

    // Always start with these two commands
    scheduledCommands.push_back({'2', 0.0f});  // Stand
    scheduledCommands.push_back({'4', 1.0f});  // Walk mode after 1s

    // Load additional commands from text file
    std::ifstream file("/home/scakki-vm/action_plan.txt");
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            std::string cmdStr;
            float timeVal;

            if (iss >> cmdStr >> timeVal) {
                if (!cmdStr.empty()) {
                    ScheduledCommand sc;
                    sc.cmd = (cmdStr == "SPACE") ? ' ' : cmdStr[0];
                    sc.time = timeVal;
                    scheduledCommands.push_back(sc);
                }
            }
        }

        // Sort all commands (including default) by time
        std::sort(scheduledCommands.begin(), scheduledCommands.end(),
            [](const ScheduledCommand& a, const ScheduledCommand& b) {
                return a.time < b.time;
            });
    } else {
        std::cerr << "Failed to open action_plan.txt" << std::endl;
    }

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
    case 'w': case 'W': userValue.ly = std::min(userValue.ly + sensitivityLeft, 1.0f); break;
    case 's': case 'S': userValue.ly = std::max(userValue.ly - sensitivityLeft, -1.0f); break;
    case 'd': case 'D': userValue.lx = std::min(userValue.lx + sensitivityLeft, 1.0f); break;
    case 'a': case 'A': userValue.lx = std::max(userValue.lx - sensitivityLeft, -1.0f); break;
    case 'i': case 'I': userValue.ry = std::min(userValue.ry + sensitivityRight, 1.0f); break;
    case 'k': case 'K': userValue.ry = std::max(userValue.ry - sensitivityRight, -1.0f); break;
    case 'l': case 'L': userValue.rx = std::min(userValue.rx + sensitivityRight, 1.0f); break;
    case 'j': case 'J': userValue.rx = std::max(userValue.rx - sensitivityRight, -1.0f); break;
    default: break;
    }
}

void* KeyBoard::runKeyBoard(void *arg) {
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg) {
    while (true) {
        // Calculate elapsed time in seconds
        float currentTime = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - startTime
        ).count();

        // Process all commands that are due
        while (!scheduledCommands.empty() && 
               scheduledCommands.front().time <= currentTime) {
            char command = scheduledCommands.front().cmd;
            scheduledCommands.pop_front();

            // Process the command
            _c = command;
            userCmd = checkCmd();
            if (userCmd == UserCommand::NONE) {
                changeValue();
            }
            _c = '\0';  // Reset after processing
        }
        
        usleep(1000);  // Sleep 1ms to prevent busy-waiting
    }
    return NULL;
}