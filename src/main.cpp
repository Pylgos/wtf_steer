#include <mbed.h>

#include <advanced_can.hpp>
#include <amt21.hpp>
#include <anglelib.hpp>
#include <c620.hpp>
#include <controller.hpp>
#include <first_penguin.hpp>
#include <pid_controller.hpp>
#include <rs485.hpp>
#include <steer_4w_controller.hpp>
#include <steer_angle_controller.hpp>
#include <steer_unit_controller.hpp>
#include <vmath.hpp>

using namespace anglelib;
using namespace vmath;


static constexpr chrono::microseconds loop_period = 10ms;
static constexpr float drive_motor_gear_ratio = 1.0 / 19.0;
static constexpr float wheel_radius = 0.05;


BufferedSerial pc{USBTX, USBRX, 115200};
Rs485 rs485{PB_6, PA_10, (int)2e6, PC_0};
AdvancedCAN can1;
AdvancedCAN can2;
Timer timer;
DigitalIn limit_sw[] = {PC_4, PC_5, PC_6, PC_7, PC_8, PC_9, PC_10, PC_11, PC_12, PC_13};

C620Array c620_array;
C620* const front_left_drive_motor = &c620_array[0];
C620* const rear_left_drive_motor = &c620_array[1];
C620* const rear_right_drive_motor = &c620_array[2];
C620* const front_right_drive_motor = &c620_array[3];
const std::array<C620*, 4> drive_motors = {
    front_left_drive_motor, rear_left_drive_motor, rear_right_drive_motor, front_right_drive_motor};

FirstPenguinArray first_penguin_array{40};
FirstPenguinArray fp_mech[] = {{30}, {35}};
FirstPenguin* const front_left_steer_motor = &first_penguin_array[2];
FirstPenguin* const rear_left_steer_motor = &first_penguin_array[1];
FirstPenguin* const rear_right_steer_motor = &first_penguin_array[0];
FirstPenguin* const front_right_steer_motor = &first_penguin_array[3];
const std::array<FirstPenguin*, 4> steer_motors = {
    front_left_steer_motor, rear_left_steer_motor, rear_right_steer_motor, front_right_steer_motor};

Amt21 front_left_steer_enc{&rs485, 0x58, -0.5, Anglef::from_deg(-74.487)};
Amt21 rear_left_steer_enc{&rs485, 0x54, -0.5, Anglef::from_deg(9.272)};
Amt21 rear_right_steer_enc{&rs485, 0x50, -0.5, Anglef::from_deg(-25.620)};
Amt21 front_right_steer_enc{&rs485, 0x5C, -0.5, Anglef::from_deg(-21.665)};
std::array<Amt21*, 4> steer_encoders = {
    &front_left_steer_enc, &rear_left_steer_enc, &rear_right_steer_enc, &front_right_steer_enc};

CircularBuffer<CANMessage, 127> can_write_buffer;
CircularBuffer<CANMessage, 127> can_read_buffer;

void can_push(const CANMessage& msg) {
  can_write_buffer.push(msg);
}

void flush_can_buffer() {
  if (!can1.isOpened()) return;
  CANMessage buffered_msg;
  while(can_write_buffer.peek(buffered_msg)) {
    if(can1.write(buffered_msg)) {
      can_write_buffer.pop(buffered_msg);
    } else {
      break;
    }
  }
}

Controller controller{can_push};

Steer4WController steer_controller{
    PidGain{.kp = 0.5, .max = 0.9, .min = -0.9}, PidGain{.kp = 1.0, .ki = 0.1, .max = 20.0, .min = -20.0}, wheel_radius,
    // PidGain{.kp = 1.0, .max = 0.9, .min = -0.9}, PidGain{.kp = 1.0, .ki = 0.1, .max = 20.0, .min = -20.0}, wheel_radius,
    Vec2f(0.201, 0.201)};

bool try_init_can() {
  bool both_initilized = true;

  if(!can1.isOpened()) {
    if(can1.init(PA_11, PA_12, (int)1e6)) {
      printf("can1 was initialized\n");
      can1.attach(
          []() {
            CANMessage msg;
            if(can1.read(msg)) {
              can_read_buffer.push(msg);
            }
          },
          RawCAN::RxIrq);
    } else {
      both_initilized = false;
    }
  }

  if(!can2.isOpened()) {
    if (can2.init(PB_12, PB_13, (int)1e6)) {
      printf("can2 was initialized\n");
    } else {
      both_initilized = false;
    }
  }

  return both_initilized;
}

void write_can() {
  try_init_can();

  if (can1.isOpened()) {
    const auto fp_msg = first_penguin_array.to_msg();
    if(!can1.write(fp_msg)) {
      printf("failed to write first penguin msg\n");
    }
    can1.write(fp_mech[0].to_msg());
    can1.write(fp_mech[1].to_msg());
  }

  if (can2.isOpened()) {
    const auto c620_msgs = c620_array.to_msgs();
    if(!can2.write(c620_msgs[0]) || !can2.write(c620_msgs[1])) {
      // printf("failed to write c620 msg\n");
    }
  }
}

void read_can() {
  try_init_can();
  CANMessage msg;

  while(can_read_buffer.pop(msg)) {
    controller.parse_packet(msg, timer.elapsed_time());
    fp_mech[0].parse_packet(msg);
    fp_mech[1].parse_packet(msg);
  }

  if (can2.isOpened()) {
    if(can2.read(msg)) {
      c620_array.parse_packet(msg);
    }
  }
}

int update_steer_encoders() {
  int error_count = 0;
  for(auto& enc: steer_encoders) {
    if(!enc->update_pos()) {
      // printf("failed to update steer encoder\n");
      error_count++;
    }
    wait_us(10);
  }
  return error_count;
}

struct Donfan {
  void task() {
    bool lim[2] = {!limit_sw[7], !limit_sw[6]};
    if(lim[0] && dir == 1) dir = 0;
    if(lim[1] && dir == -1) dir = 0;

    if(dir == 1) {
      fp_mech[1][1].set_raw_duty(-8000);
    } else if(dir == -1) {
      fp_mech[1][1].set_raw_duty(8000);
    } else {
      fp_mech[1][1].set_raw_duty(0);
    }
  }
  int8_t dir = 0;
} donfan;
struct Expander {
  void task() {
    // 下10 上6
    // bool lim[2] = {!limit_sw[5], !limit_sw[9]};
    auto now = HighResClock::now();
    pid.update(-fp_mech[0][3].get_enc(), now - pre);
    fp_mech[0][3].set_duty(-pid.get_output());
    pre = now;
  }
  PidController pid = {PidGain{.kp = 0.0005, .max = 0.9, .min = -0.9}};
  decltype(HighResClock::now()) pre = HighResClock::now();
} expander;
struct Collector {
  void task() {
    // 3
    bool lim = !limit_sw[2];
    if(collecting) {
      state = Running;
    } else if(state == Stop || lim) {
      state = Stop;
    } else {
      state = Storing;
    }

    switch(state) {
      case Stop:
        fp_mech[1][0].set_raw_duty(0);
        break;
      case Running:
      case Storing:
        fp_mech[1][0].set_raw_duty(8000);
        break;
    }
  }
  enum {
    Stop,
    Running,
    Storing,
  } state;
  bool collecting;
} collector;
struct ArmAngle {
  void task() {
    // 4
    bool lim = !limit_sw[3];
    if(state == Waiting && lim) {
      origin = c620_array[4].get_ang_vel();
      state = Running;
    }
    auto now = HighResClock::now();
    if(state == Running) {
      pid.update(c620_array[4].get_ang_vel() - origin, now - pre);
      c620_array[4].set_raw_tgt_current(pid.get_output());
    }
    pre = now;
  }
  enum {
    Waiting,
    Running,
  } state;
  PidController pid = {PidGain{.kp = 1.0, .max = 0.9, .min = -0.9}};
  decltype(HighResClock::now()) pre = HighResClock::now();
  float origin = 0;
} arm_angle;
struct ArmLength {
  void task() {
    // bool lim = !limit_sw[8];
    auto now = HighResClock::now();
    if(now - pre < 100ms) {
      fp_mech[0][2].set_duty(duty);
    } else {
      fp_mech[0][2].set_duty(0);
    }
    pre = now;
  }
  decltype(HighResClock::now()) pre = HighResClock::now();
  int16_t duty = 0;
} arm_length;
// struct ArmLength {
//   void task() {
//     // 9
//     bool lim = !limit_sw[8];
//     if(state == Waiting && lim) {
//       origin = fp_mech[0][2].get_enc();
//       state = Running;
//     }
//     auto now = HighResClock::now();
//     if(state == Running) {
//       pid.update(fp_mech[0][2].get_enc(), now - pre);
//       fp_mech[0][2].set_duty(pid.get_output());
//     }
//     pre = now;
//   }
//   enum {
//     Waiting,
//     Running,
//   } state;
//   PidController pid = {PidGain{.kp = 0.1, .max = 0.9, .min = -0.9}};
//   decltype(HighResClock::now()) pre = HighResClock::now();
//   float origin = 0;
// } arm_length;
struct LargeWheel {
  void task() {
    fp_mech[0][0].set_raw_duty(duty);
    fp_mech[0][1].set_raw_duty(-duty);
    fp_mech[1][2].set_raw_duty(duty);
    fp_mech[1][3].set_raw_duty(-duty);
  }
  int16_t duty;
} large_wheel;

void mech_task() {
  donfan.task();
  expander.task();
  collector.task();
  arm_length.task();
  large_wheel.task();
}

int main() {
  printf("start\n");

  try_init_can();

  controller.on_reset_pid([]() {
    printf("pid reset\n");
    // steer_controller.set_steer_gain(controller.get_steer_gain());
    // steer_controller.set_drive_gain(controller.get_drive_gain());
    steer_controller.reset();
  });

  controller.on_activation([]() {
    printf("activated\n");
  });

  controller.on_deactivation([]() {
    printf("deactivated\n");
    // controller.activate();
  });

  controller.on_steer_offset([](int idx, Anglef offset) {
    printf("steer offset: %d %7.3f\n", idx, offset.rad());
    steer_controller.set_steer_offset(idx, offset);
  });

  controller.on_unwind([]() {
    printf("unwind\n");
    steer_controller.on_unwound([](bool success) {
      printf("unwind done\n");
      controller.publish_steer_unwind_done();
    });
    steer_controller.start_unwinding();
  });

  controller.on_donfan([](int8_t dir) {
    printf("donfan % 2d\n", dir);
    donfan.dir = dir;
  });
  controller.on_expander([](int16_t height) {
    expander.pid.set_target(height);
    printf("expander %d\n", height);
  });
  controller.on_collector([](bool collect) {
    printf("collector %1d\n", collect);
    collector.collecting = collect;
  });
  controller.on_arm_angle([](int16_t angle) {
    printf("arm_angle %d\n", angle);
    arm_angle.pid.set_target(angle);
  });
  controller.on_arm_length([](int16_t length) {
    printf("arm_length %d\n", length);
    arm_length.pre = HighResClock::now();
    static auto pre = length;
    arm_length.duty = (length - pre) * 20;
    pre = length;
    // arm_length.pid.set_target(length);
  });
  controller.on_large_wheel([](int16_t duty) {
    printf("large_wheel %d\n", duty);
    large_wheel.duty = duty;
  });

  front_left_drive_motor->set_gear_ratio(-drive_motor_gear_ratio);
  rear_left_drive_motor->set_gear_ratio(-drive_motor_gear_ratio);
  rear_right_drive_motor->set_gear_ratio(drive_motor_gear_ratio);
  front_right_drive_motor->set_gear_ratio(drive_motor_gear_ratio);

  for(const auto mot: steer_motors) {
    mot->set_invert(true);
    // mot->set_invert(false);
  }

  timer.start();

  Timer dt_timer;
  dt_timer.start();
  ThisThread::sleep_for(chrono::duration_cast<Kernel::Clock::duration_u32>(loop_period));

  controller.activate();

  while(true) {
    std::chrono::microseconds dt = dt_timer.elapsed_time();
    dt_timer.reset();

    read_can();
    int error_count = update_steer_encoders();
    if(error_count > 0) {
      continue;
    }
    controller.update(timer.elapsed_time());

    // for(size_t i = 0; i < 4; i++) {
    //   printf(" %f", steer_encoders[i]->get_angle().deg());
    // }
    // printf("\n");

    switch(controller.get_state()) {
      case Feedback::CurrentState::CONFIGURING: {
        steer_controller.reset();
        for(size_t i = 0; i < 4; i++) {
          drive_motors[i]->set_tgt_torque(0);
          steer_motors[i]->set_duty(0);
        }
      } break;

      case Feedback::CurrentState::RUNNING: {
        // auto tgt_vel = controller.get_tgt_linear_vel();
        // auto tgt_ang = controller.get_tgt_ang_vel();
        // printf("% 5f ", tgt_vel.x);
        // printf("% 5f ", tgt_vel.y);
        // printf("% 5f ", tgt_ang);

        // for(auto& e: fp_mech[0]) printf("% 6d ", e.get_raw_duty());
        // for(auto& e: fp_mech[1]) printf("% 6d ", e.get_raw_duty());

        steer_controller.set_tgt_vel(controller.get_tgt_linear_vel(), controller.get_tgt_ang_vel());

        steer_controller.update({drive_motors[0]->get_ang_vel(), drive_motors[1]->get_ang_vel(),
                                 drive_motors[2]->get_ang_vel(), drive_motors[3]->get_ang_vel()},
                                {steer_encoders[0]->get_angle(), steer_encoders[1]->get_angle(),
                                 steer_encoders[2]->get_angle(), steer_encoders[3]->get_angle()},
                                dt);

        auto drive_cmd = steer_controller.get_drive_outputs();
        auto steer_cmd = steer_controller.get_steer_outputs();

        for(size_t i = 0; i < 4; i++) {
          // printf("% 5f ", drive_cmd[i]);
          drive_motors[i]->set_tgt_torque(drive_cmd[i]);
          steer_motors[i]->set_duty(steer_cmd[i]);
        }

        printf("% 6ld ", fp_mech[0][3].get_enc());
        for(auto& e: fp_mech[0]) printf("% 6d ", e.get_raw_duty());
        for(auto& e: fp_mech[1]) printf("% 6d ", e.get_raw_duty());
        // for(auto& e: c620_array) printf("% 6d ", e.get_raw_tgt_current());
        printf("\n");

        controller.set_odom(steer_controller.get_odom_linear_vel(), steer_controller.get_odom_ang_vel());
      } break;
    }

    mech_task();

    // for(auto& e: steer_motors) {
    //   e->set_duty(0);
    // }
    // steer_motors[3]->set_duty(0.5);
    write_can();

    // for(auto& e: steer_encoders) {
    //   printf("% 6f ", e->get_angle().rad());
    // }
    // for(auto& e: limit_sw) {
    //   printf("%d ", e.read());
    // }

    // for(auto& e: fp_mech[0]) printf("% 6d ", e.get_raw_duty());
    // for(auto& e: fp_mech[1]) printf("% 6d ", e.get_raw_duty());
    // printf("\n");

    do {
      read_can();
      flush_can_buffer();
    } while(loop_period > dt_timer.elapsed_time());
  }
}
