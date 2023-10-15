#include <mbed.h>

#include <advanced_can.hpp>
#include <amt21.hpp>
#include <anglelib.hpp>
#include <bno055.hpp>
#include <c620.hpp>
#include <controller.hpp>
#include <first_penguin.hpp>
#include <mechanism.hpp>
#include <pid_controller.hpp>
#include <rs485.hpp>
#include <steer_4w_controller.hpp>
#include <steer_angle_controller.hpp>
#include <steer_unit_controller.hpp>
#include <vmath.hpp>

using namespace anglelib;
using namespace vmath;


static constexpr chrono::microseconds loop_period = 1ms;
static constexpr float drive_motor_gear_ratio = 1.0 / 19.0;
static constexpr float wheel_radius = 0.05;


BufferedSerial pc{USBTX, USBRX, 115200};
Rs485 rs485{PB_6, PA_10, (int)2e6, PC_0};
AdvancedCAN can1;
AdvancedCAN can2;
Timer timer;
Bno055 bno055{PA_0, PA_1};
DigitalIn limit_sw[] = {PC_4, PC_5, PC_6, PC_7, PC_8, PC_9, PC_10, PC_11, PC_12, PC_13};

C620Array c620_array;
C620* const front_left_drive_motor = &c620_array[2];
C620* const rear_left_drive_motor = &c620_array[3];
C620* const rear_right_drive_motor = &c620_array[0];
C620* const front_right_drive_motor = &c620_array[1];
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

Amt21 front_left_steer_enc{&rs485, 0x58, -1.0, Anglef::from_deg(-1.2 - 1.8 + 0.9)};
Amt21 rear_left_steer_enc{&rs485, 0x50, -1.0, Anglef::from_deg(70.8 + 1.3 + 0.7 - 4.8 + 2.5 - 1.0)};
Amt21 rear_right_steer_enc{&rs485, 0x54, -1.0, Anglef::from_deg(26.4)};
Amt21 front_right_steer_enc{&rs485, 0x5C, -1.0, Anglef::from_deg(3.3 + 0.4)};
std::array<Amt21*, 4> steer_encoders = {
    &front_left_steer_enc, &rear_left_steer_enc, &rear_right_steer_enc, &front_right_steer_enc};

CircularBuffer<CANMessage, 127> can_write_buffer;
CircularBuffer<CANMessage, 127> can_read_buffer;

void can_push(const CANMessage& msg) {
  can_write_buffer.push(msg);
}

void flush_can_buffer() {
  if(!can1.isOpened()) return;
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

Steer4WController steer_controller{PidGain{}, PidGain{}, wheel_radius, Vec2f(0.201, 0.201)};

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
    if(can2.init(PB_12, PB_13, (int)1e6)) {
      printf("can2 was initialized\n");
    } else {
      both_initilized = false;
    }
  }

  return both_initilized;
}

void write_can() {
  try_init_can();

  if(can1.isOpened()) {
    const auto fp_msg = first_penguin_array.to_msg();
    if(!can1.write(fp_msg) || !can1.write(fp_mech[0].to_msg()) || !can1.write(fp_mech[1].to_msg())) {
      printf("failed to write first penguin msg\n");
    }
  }

  if(can2.isOpened()) {
    const auto c620_msgs = c620_array.to_msgs();
    if(!can2.write(c620_msgs[0]) || !can2.write(c620_msgs[1])) {
      // printf("failed to write c620 msg\n");
    }
  }
}

static HighResClock::time_point last_c620 = {};
void read_can() {
  try_init_can();
  CANMessage msg;

  while(can_read_buffer.pop(msg)) {
    controller.parse_packet(msg, timer.elapsed_time());
    fp_mech[0].parse_packet(msg);
    fp_mech[1].parse_packet(msg);
  }

  if(can2.isOpened()) {
    if(can2.read(msg) && c620_array.parse_packet(msg)) {
      last_c620 = HighResClock::now();
    }
  }
}

int update_steer_encoders() {
  int error_count = 0;
  for(auto& enc: steer_encoders) {
    if(!enc->update_pos()) {
      printf("failed to update steer encoder\n");
      error_count++;
    }
    wait_us(10);
  }
  return error_count;
}

/// @brief bno055から姿勢角を取得
/// @return 1 if success, 0 otherwise.
bool update_gyro() {
  return bno055.request_euler_angle() == Bno055::Response::WRITE_SUCCESS;
}

Mechanism mech = {
    .donfan = {.fp = &fp_mech[1][1], .lim_fwd = &limit_sw[7], .lim_rev = &limit_sw[6]},
    .expander = {.fp = &fp_mech[0][3], .lim = &limit_sw[9]},
    .collector = {.fp = &fp_mech[1][0], .lim = &limit_sw[2]},
    .arm_angle = {.c620 = &c620_array[4], .fp = &fp_mech[1][0], .lim = &limit_sw[3]},
    .arm_length = {.fp = &fp_mech[0][2], .lim = &limit_sw[8]},
    .large_wheel = {.c620_arr = {&c620_array[5], &c620_array[6]}},
};

int main() {
  printf("start\n");

  try_init_can();
  bno055.init();

  controller.on_reset_pid([]() {
    printf("pid reset\n");
    steer_controller.set_steer_gain(controller.get_steer_gain());
    steer_controller.set_drive_gain(controller.get_drive_gain());
    mech.set_arm_length_gain(controller.get_arm_length_gain());
    mech.set_arm_angle_gain(controller.get_arm_angle_gain());
    mech.set_expander_gain(controller.get_expander_gain());
    steer_controller.reset();
  });

  controller.on_activation([]() {
    printf("activated\n");
  });

  controller.on_deactivation([]() {
    printf("deactivated\n");
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
    mech.donfan.dir = dir;
  });
  controller.on_expander([](int16_t height) {
    mech.expander.set_target(height);
    printf("expander %d\n", height);
  });
  controller.on_collector([](bool collect) {
    printf("collector %1d\n", collect);
    mech.collector.collecting = collect;
  });
  controller.on_arm_angle([](int16_t angle) {
    printf("arm_angle %d\n", angle);
    mech.arm_angle.set_target(angle);
  });
  controller.on_arm_length([](int16_t length) {
    printf("arm_length %d\n", length);
    mech.arm_length.set_target(length);
  });
  controller.on_large_wheel([](int16_t duty) {
    printf("large_wheel %d\n", duty);
    mech.large_wheel.tag_duty = duty;
  });
  controller.publish_steer_state([](int idx) {
    Feedback::SteerUnitState steer_state;
    steer_state.index = idx;
    steer_state.velocity = drive_motors[idx]->get_ang_vel() * 1e2;
    steer_state.current = drive_motors[idx]->get_actual_current() * 1e3;
    steer_state.angle = steer_encoders[idx]->get_angle().direction().rad() * 1e3;
    return steer_state;
  });

  front_left_drive_motor->set_gear_ratio(-drive_motor_gear_ratio);
  rear_left_drive_motor->set_gear_ratio(-drive_motor_gear_ratio);
  rear_right_drive_motor->set_gear_ratio(drive_motor_gear_ratio);
  front_right_drive_motor->set_gear_ratio(drive_motor_gear_ratio);

  for(const auto mot: steer_motors) {
    mot->set_invert(true);
  }

  timer.start();

  Timer dt_timer;
  dt_timer.start();
  ThisThread::sleep_for(chrono::duration_cast<Kernel::Clock::duration_u32>(loop_period));

  // controller.activate();

  while(true) {
    std::chrono::microseconds dt = dt_timer.elapsed_time();
    dt_timer.reset();

    read_can();
    int error_count = update_steer_encoders();
    if(error_count > 0) {
      continue;
    }
    if(!update_gyro()) continue;
    controller.update(timer.elapsed_time());

    for(size_t i = 0; i < 4; i++) {
      printf("% 6.1f ", steer_encoders[i]->get_angle().deg());
    }

    switch(controller.get_state()) {
      case Feedback::CurrentState::CONFIGURING: {
        steer_controller.reset();
        for(size_t i = 0; i < 4; i++) {
          drive_motors[i]->set_tgt_torque(0);
          steer_motors[i]->set_duty(0);
        }
      } break;

      case Feedback::CurrentState::RUNNING: {
        steer_controller.set_tgt_vel(controller.get_tgt_linear_vel(), controller.get_tgt_ang_vel());

        auto now = HighResClock::now();
        if(now - last_c620 > 100ms) {
          printf("detect emergency.");
          steer_controller.reset();
        } else {
          steer_controller.update({drive_motors[0]->get_ang_vel(), drive_motors[1]->get_ang_vel(),
                                   drive_motors[2]->get_ang_vel(), drive_motors[3]->get_ang_vel()},
                                  {steer_encoders[0]->get_angle(), steer_encoders[1]->get_angle(),
                                   steer_encoders[2]->get_angle(), steer_encoders[3]->get_angle()},
                                  dt);
        }

        auto drive_cmd = steer_controller.get_drive_outputs();
        auto steer_cmd = steer_controller.get_steer_outputs();

        for(size_t i = 0; i < 4; i++) {
          drive_motors[i]->set_tgt_torque(drive_cmd[i]);
          steer_motors[i]->set_duty(steer_cmd[i]);
        }
        for(auto& e: steer_motors) printf("% 6d ", e->get_raw_duty());

        controller.set_vel(steer_controller.get_odom_linear_vel(), steer_controller.get_odom_ang_vel());
        controller.set_pose(steer_controller.get_odom_linear_pose(), bno055.get_x_rad());
      } break;
    }

    mech.task();
    write_can();
    printf("\n");

    do {
      read_can();
      flush_can_buffer();
    } while(loop_period > dt_timer.elapsed_time());
  }
}
