#include <mbed.h>
#include <anglelib.hpp>
#include <rs485.hpp>
#include <amt21.hpp>
#include <c620.hpp>
#include <first_penguin.hpp>
#include <pid_controller.hpp>
#include <steer_angle_controller.hpp>
#include <steer_unit_controller.hpp>
#include <steer_4w_controller.hpp>
#include <controller.hpp>
#include <vmath.hpp>

using namespace anglelib;
using namespace vmath;


static constexpr chrono::microseconds loop_period = 1ms;
static constexpr float drive_motor_gear_ratio = 1.0 / 19.0;
static constexpr float wheel_radius = 0.045;


BufferedSerial pc{USBTX, USBRX, 115200};
Rs485 rs485{PB_6, PA_10, (int)2e6, PC_0};
CAN can1{PA_11, PA_12, (int)1e6};
CAN can2{PB_12, PB_13, (int)1e6};
Timer timer;

C620Array c620_array;
C620* const front_left_drive_motor = &c620_array[0];
C620* const rear_left_drive_motor = &c620_array[1];
C620* const rear_right_drive_motor = &c620_array[2];
C620* const front_right_drive_motor = &c620_array[3];
const std::array<C620*, 4> drive_motors = {front_left_drive_motor, rear_left_drive_motor, rear_right_drive_motor, front_right_drive_motor};

FirstPenguinArray first_penguin_array{5};
FirstPenguin* const front_left_steer_motor = &first_penguin_array[0];
FirstPenguin* const rear_left_steer_motor = &first_penguin_array[1];
FirstPenguin* const rear_right_steer_motor = &first_penguin_array[2];
FirstPenguin* const front_right_steer_motor = &first_penguin_array[3];
const std::array<FirstPenguin*, 4> steer_motors = {front_left_steer_motor, rear_left_steer_motor, rear_right_steer_motor, front_right_steer_motor};

Amt21 front_left_steer_enc{&rs485, 0x50, -0.5, -Anglef::from_deg(45)};
Amt21 rear_left_steer_enc{&rs485, 0x54, -0.5, Anglef::from_deg(45)};
Amt21 rear_right_steer_enc{&rs485, 0x58, -0.5, Anglef::from_deg(45)};
Amt21 front_right_steer_enc{&rs485, 0x5C, -0.5, Anglef::from_deg(45)};
std::array<Amt21*, 4> steer_encoders = {&front_left_steer_enc, &rear_left_steer_enc, &rear_right_steer_enc, &front_right_steer_enc};

CircularBuffer<CANMessage, 127> can_write_buffer;

void can_push(const CANMessage& msg) {
  can_write_buffer.push(msg);
}

void flush_can_buffer() {
  CANMessage buffered_msg;
  while (can_write_buffer.peek(buffered_msg)) {
    if (can1.write(buffered_msg)) {
      can_write_buffer.pop(buffered_msg);
    } else {
      break;
    }
  }
}

Controller controller{can_push};

Steer4WController steer_controller{PidGain{}, PidGain{}, wheel_radius, Vec2f(0.5, 0.5)};

void write_can() {
  const auto fp_msg = first_penguin_array.to_msg();
  if (!can1.write(fp_msg)) {
    printf("failed to write first penguin msg\n");
  }

  const auto c620_msgs = c620_array.to_msgs();
  if (!can2.write(c620_msgs[0]) || !can2.write(c620_msgs[1])) {
    printf("failed to write c620 msg\n");
  }
}

void read_can() {
  CANMessage msg;

  if (can1.read(msg)) {
    controller.parse_packet(msg, timer.elapsed_time());
  }

  if (can2.read(msg)) {
    c620_array.parse_packet(msg);
  }
}

void update_steer_encoders() {
  for (auto& enc : steer_encoders) {
    if (!enc->update_pos()) {
      printf("failed to update steer encoder\n");
    }
    wait_us(10);
  }
}

int main() {
  printf("start\n");

  controller.on_reset_pid([](){
    printf("pid reset\n");
    steer_controller.set_steer_gain(controller.get_steer_gain());
    steer_controller.set_velocity_gain(controller.get_velocity_gain());
    steer_controller.reset();
  });

  controller.on_activation([](){
    printf("activated\n");
  });

  controller.on_deactivation([](){
    printf("deactivated\n");
  });

  controller.on_steer_offset([](int idx, Anglef offset){
    printf("steer offset: %d %7.3f\n", idx, offset.rad());
    steer_controller.set_steer_offset(idx, offset);
  });

  front_left_drive_motor->set_gear_ratio(-drive_motor_gear_ratio);
  rear_left_drive_motor->set_gear_ratio(-drive_motor_gear_ratio);
  rear_right_drive_motor->set_gear_ratio(drive_motor_gear_ratio);
  front_right_drive_motor->set_gear_ratio(drive_motor_gear_ratio);

  for (const auto mot : steer_motors) {
    mot->set_invert(true);
  }

  timer.start();

  Timer dt_timer;
  dt_timer.start();
  ThisThread::sleep_for(chrono::duration_cast<Kernel::Clock::duration_u32>(loop_period));

  while (true) {
    std::chrono::microseconds dt = dt_timer.elapsed_time();
    dt_timer.reset();

    read_can();
    update_steer_encoders();
    controller.update(timer.elapsed_time());

    switch (controller.get_state()) {
      case Feedback::CurrentState::CONFIGURING: {
        steer_controller.reset();
        for (size_t i = 0; i < 4; i++) {
          drive_motors[i]->set_tgt_torque(0);
          steer_motors[i]->set_duty(0);
        }
      } break;

      case Feedback::CurrentState::RUNNING: {
        steer_controller.set_tgt_vel(
          controller.get_tgt_linear_vel(),
          controller.get_tgt_ang_vel());

        steer_controller.update(
          {drive_motors[0]->get_ang_vel(), drive_motors[1]->get_ang_vel(), drive_motors[2]->get_ang_vel(), drive_motors[3]->get_ang_vel()},
          {steer_encoders[0]->get_angle(), steer_encoders[1]->get_angle(), steer_encoders[2]->get_angle(), steer_encoders[3]->get_angle()},
          dt);

        auto drive_cmd = steer_controller.get_drive_outputs();
        auto steer_cmd = steer_controller.get_steer_outputs();

        for (size_t i = 0; i < 4; i++) {
          drive_motors[i]->set_tgt_torque(drive_cmd[i]);
          steer_motors[i]->set_duty(steer_cmd[i]);
        }

        controller.set_odom(
          steer_controller.get_odom_linear_vel(),
          steer_controller.get_odom_ang_vel());
      } break;
    }

    write_can();

    do {
      flush_can_buffer();
    } while (loop_period > dt_timer.elapsed_time());
  }
}
