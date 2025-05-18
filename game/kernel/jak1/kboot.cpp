/*!
 * @file kboot.cpp
 * GOAL Boot.  Contains the "main" function to launch GOAL runtime
 * DONE!
 */

#include "kboot.h"

#include <chrono>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "game/system/hid/devices/game_controller.h"
#include "common/common_types.h"
#include "common/log/log.h"
#include "common/util/Timer.h"

#include "game/common/game_common_types.h"
#include "game/kernel/common/klisten.h"
#include "game/kernel/common/kprint.h"
#include "game/kernel/common/kscheme.h"
#include "game/kernel/common/ksocket.h"
#include "game/kernel/jak1/klisten.h"
#include "game/kernel/jak1/kmachine.h"
#include "game/sce/libscf.h"
#include <libsm64/libsm64.h>


using namespace ee;

namespace jak1 {
VideoMode BootVideoMode;

void kboot_init_globals() {}

/*!
 * Launch the GOAL Kernel (EE).
 * DONE!
 * See InitParms for launch argument details.
 * @param argc : argument count
 * @param argv : argument list
 * @return 0 on success, otherwise failure.
 *
 * CHANGES:
 * Added InitParms call to handle command line arguments
 * Removed hard-coded debug mode disable
 * Renamed from `main` to `goal_main`
 * Add call to sceDeci2Reset when GOAL shuts down.
 */
s32 goal_main(int argc, const char* const* argv) {
  // Initialize global variables based on command line parameters
  // This call is not present in the retail version of the game
  // but the function is, and it likely goes here.
  InitParms(argc, argv);

  // Initialize CRC32 table for string hashing
  init_crc();

  // NTSC V1, NTSC v2, PAL CD Demo, PAL Retail
  // Set up game configurations
  masterConfig.aspect = (u16)sceScfGetAspect();
  masterConfig.language = (u16)sceScfGetLanguage();
  masterConfig.inactive_timeout = 0;  // demo thing
  masterConfig.timeout = 0;           // demo thing
  masterConfig.volume = 100;

  // Set up language configuration
  if (masterConfig.language == SCE_SPANISH_LANGUAGE) {
    masterConfig.language = (u16)Language::Spanish;
  } else if (masterConfig.language == SCE_FRENCH_LANGUAGE) {
    masterConfig.language = (u16)Language::French;
  } else if (masterConfig.language == SCE_GERMAN_LANGUAGE) {
    masterConfig.language = (u16)Language::German;
  } else if (masterConfig.language == SCE_ITALIAN_LANGUAGE) {
    masterConfig.language = (u16)Language::Italian;
  } else if (masterConfig.language == SCE_JAPANESE_LANGUAGE) {
    // Note: this case was added so it is easier to test Japanese fonts.
    masterConfig.language = (u16)Language::Japanese;
  } else {
    // pick english by default, if language is not supported.
    masterConfig.language = (u16)Language::English;
  }

  // Set up aspect ratio override in demo
  if (!strcmp(DebugBootMessage, "demo") || !strcmp(DebugBootMessage, "demo-shared")) {
    masterConfig.aspect = SCE_ASPECT_FULL;
  }

  // In retail game, disable debugging modes, and force on DiskBoot
  // MasterDebug = 0;
  // DiskBoot = 1;
  // DebugSegment = 0;

  // Launch GOAL!

  if (InitMachine() >= 0) {    // init kernel
    KernelCheckAndDispatch();  // run kernel
    ShutdownMachine();         // kernel died, we should too.
  } else {
    fprintf(stderr, "InitMachine failed\n");
    exit(1);
  }

  return 0;
}

/*!
 * Main loop to dispatch the GOAL kernel.
 */

 int fartmarioId2 = 0;

void KernelCheckAndDispatch() {
  u64 goal_stack = u64(g_ee_main_mem) + EE_MAIN_MEM_SIZE - 8;

        // Must call this once with a pointer to SM64 ROM data
    // Read the rom data (make sure it's an unmodified SM64 US ROM)
    FILE* romFile = fopen("sm64.us.z64", "rb");
    fseek(romFile, 0, SEEK_END);
    size_t romSize = ftell(romFile);
    rewind(romFile);
    uint8_t* romData = new uint8_t[romSize];
    fread(romData, 1, romSize, romFile);
    fclose(romFile);

    // if (sm64_init(romData, romSize) != 0) {
    //     fprintf(stderr, "Failed to init SM64\n");
    //     return -1;
    // }
    delete[] romData;


    SM64Surface surfaces[2];

// Triangle 1
surfaces[0].vertices[0][0] = -1000;
surfaces[0].vertices[0][1] = 0;
surfaces[0].vertices[0][2] = -1000;
surfaces[0].vertices[1][0] = 1000;
surfaces[0].vertices[1][1] = 0;
surfaces[0].vertices[1][2] = -1000;
surfaces[0].vertices[2][0] = 1000;
surfaces[0].vertices[2][1] = 0;
surfaces[0].vertices[2][2] = 1000;

// Triangle 2
surfaces[1].vertices[0][0] = -1000;
surfaces[1].vertices[0][1] = 0;
surfaces[1].vertices[0][2] = -1000;
surfaces[1].vertices[1][0] = 1000;
surfaces[1].vertices[1][1] = 0;
surfaces[1].vertices[1][2] = 1000;
surfaces[1].vertices[2][0] = -1000;
surfaces[1].vertices[2][1] = 0;
surfaces[1].vertices[2][2] = 1000;

sm64_static_surfaces_load(surfaces, 2);
fartmarioId2 = sm64_mario_create(1, 1, 1);




  while (MasterExit == RuntimeExitStatus::RUNNING) {


    static float last_position[3] = {999999, 999999, 999999};  // Init to dummy impossible state

    SM64MarioState state;
    SM64MarioGeometryBuffers geom;
    sm64_mario_tick(fartmarioId2, &m_mario_inputs, &state, &geom);
    
    //Debug: print pointer and contents of m_mario_inputs
// printf("Inputs [%p] - Stick: (%d, %d)  CamLook: (%d, %d)  A:%d B:%d Z:%d\n",
//   (void*)&m_mario_inputs,
//   m_mario_inputs.stickX, m_mario_inputs.stickY,
//   m_mario_inputs.camLookX, m_mario_inputs.camLookZ,
//   m_mario_inputs.buttonA, m_mario_inputs.buttonB, m_mario_inputs.buttonZ);
    // Check if any position changed
    //fmt::print("Inputs - Stick: ({}, {})\n", m_mario_inputs.stickX, m_mario_inputs.stickY);
    bool changed = false;
    for (int i = 0; i < 3; ++i) {
        if (state.position[i] != last_position[i]) {
            changed = true;
            break;
        }
    }
    
    if (changed) {
        printf("Mario: %f %f %f\n", state.position[0], state.position[1], state.position[2]);
        for (int i = 0; i < 3; ++i)
            last_position[i] = state.position[i];
    }
    //printf("Action: %08X\n", state.action);
    if (state.action == 0x00450045) {
      printf("Detected stuck action 0x00450045, resetting inputs and state\n");
  
      m_mario_inputs = {
          .camLookX = 0.0f,
          .camLookZ = 1.0f,
          .stickX = 0,
          .stickY = 0,
          .buttonA = 0,
          .buttonB = 0,
          .buttonZ = 0
      };
  
      sm64_set_mario_action(fartmarioId2, 0x0000000F);  // ACT_IDLE
  }
  
    // try to get a message from the listener, and process it if needed
    Ptr<char> new_message = WaitForMessageAndAck();
    if (new_message.offset) {
      ProcessListenerMessage(new_message);
    }

    // remember the old listener function
    auto old_listener = ListenerFunction->value;
    // dispatch the kernel
    //(**kernel_dispatcher)();

    Timer kernel_dispatch_timer;
    if (MasterUseKernel) {
      // use the GOAL kernel.
      call_goal_on_stack(Ptr<Function>(kernel_dispatcher->value), goal_stack, s7.offset,
                         g_ee_main_mem);
    } else {
      // use a hack to just run the listener function if there's no GOAL kernel.
      if (ListenerFunction->value != s7.offset) {
        auto result = call_goal_on_stack(Ptr<Function>(ListenerFunction->value), goal_stack,
                                         s7.offset, g_ee_main_mem);
#ifdef __linux__
        cprintf("%ld\n", result);
#else
        cprintf("%lld\n", result);
#endif
        ListenerFunction->value = s7.offset;
      }
    }

    auto time_ms = kernel_dispatch_timer.getMs();
    if (time_ms > 50) {
      lg::print("Kernel dispatch time: {:.3f} ms\n", time_ms);
    }

    ClearPending();

    // if the listener function changed, it means the kernel ran it, so we should notify compiler.
    if (MasterDebug && ListenerFunction->value != old_listener) {
      SendAck();
    }

    if (time_ms < 4) {
      std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

  
  }
}

/*!
 * Stop running the GOAL Kernel.
 * DONE, EXACT
 */
void KernelShutdown() {
  MasterExit = RuntimeExitStatus::EXIT;  // GOAL Kernel Dispatch loop will stop now.
}
}  // namespace jak1
