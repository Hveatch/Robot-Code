from XRPLib.defaults import *
from GFULib.defaults import *
from GFULib.color import *
from machine import Pin

led = Pin(8, Pin.OUT)

class StateMachine:
    state = "WAIT_FOR_GREEN"
    statename = ""

    loop_time_ms = 50

    nominal_effort = 0.4
    left_effort = nominal_effort
    right_effort = nominal_effort
    max_effort = 0.5
    min_effort = 0.25
    effort_incr_per_second = 0.2
    effort_incr_per_loop = effort_incr_per_second / loop_time_ms

    proximity_center = 500
    proximity_range_far = 300
    proximity_range_close = 800
    #error = (proximity_center - proximity_range)

    distAhead = 0
    prox_right = 0
    prox_left = 0
    heading = 0
    #old_heading = None
    target_heading = None
    turn_angle = 180
    turn_direction = "right"
    left_distance = 0
    right_distance = 0
    detected_color = "unknown"
    backup_start_position = None
    #backup_start_position = 0
    backup_distance_cm = 0.5
    door_counter = 0
    old_state = None
    forward_dist_door = 0

    update_time = {}

    def __init__(self):
        current_time = time.ticks_ms()
        self.update_time["proximity_left"] = current_time
        self.update_time["proximity_right"] = time.ticks_add(current_time, 2)
        self.update_time["heading"] = time.ticks_add(current_time, 4)
        self.update_time["rangefinder"] = time.ticks_add(current_time, 6)
        self.update_time["state_interval"] = time.ticks_add(current_time, 20)

    def print_state(self, newstate):
        if newstate != self.statename:
            #print(f"entering state {newstate}")
            self.statename = newstate

    def update_sensors(self):
        current_time = time.ticks_ms()

        if time.ticks_diff(current_time, self.update_time["proximity_right"]) >= self.loop_time_ms:
            self.update_time["proximity_right"] += self.loop_time_ms
            self.prox_right = proximity_right.getProximity()
        if time.ticks_diff(current_time, self.update_time["proximity_left"]) >= self.loop_time_ms:
            self.update_time["proximity_left"] += self.loop_time_ms
            self.prox_left = proximity_left.getProximity()
        if time.ticks_diff(current_time, self.update_time["heading"]) >= self.loop_time_ms:
            self.update_time["heading"] += self.loop_time_ms
            self.heading = imu.get_heading()
        if time.ticks_diff(current_time, self.update_time["rangefinder"]) >= self.loop_time_ms:
            self.update_time["rangefinder"] += self.loop_time_ms
            self.distAhead = rangefinder.distance()

        self.left_distance = left_motor.get_position()
        self.right_distance = right_motor.get_position()
        self.detected_color = color.getColor()

    def evaluate_state(self):
        current_time = time.ticks_ms()

        if self.state == "WAIT_FOR_GREEN":
            self.left_effort = 0
            self.right_effort = 0
            led.on()
            #print("Waiting for green light... Detected color:", self.detected_color)
            if self.detected_color == "green":
                #print("Green light detected! Turning LED off. Starting.")
                self.state = "START_FORWARD"

        elif self.state == "RESET":
            self.state = "START_FORWARD"

        elif self.state == "START_FORWARD":
            self.left_effort = 0
            self.right_effort = 0
            self.backup_start_position = (self.left_distance + self.right_distance) / 2
            led.off()
            self.state = "BACK_UP"

        elif self.state == "BACK_UP":
            self.left_effort = -0.3
            self.right_effort = -0.3
            current_position = (self.left_distance + self.right_distance) / 2
            distance_moved = abs(current_position)
            if distance_moved > self.backup_distance_cm:
                #print("Back up complete.")
                self.left_effort = 0
                self.right_effort = 0
                self.turn_angle = 180
                if self.prox_left > self.prox_right:
                    self.turn_direction = "right"
                elif self.prox_right > self.prox_left:
                    self.turn_direction = "left"
                self.state = "TURN"

        elif self.state == "FOLLOW_LEFT_WALL":
            self.right_effort = self.nominal_effort
            self.left_effort = self.nominal_effort
#            if self.prox_left <= 30:
#                self.door_counter += 1
#                if self.door_counter == 4:
#                    self.state = "DOOR"
#                    self.old_state = "FOLLOW_LEFT_WALL"
            if self.distAhead <= 10.0:
                self.turn_angle = 90
                self.turn_direction = "right"
                self.state = "TURN"
            elif self.prox_left > (self.proximity_center + self.proximity_range_close):
                self.state = "VEER_AWAY_FROM_LEFT_WALL"
            elif self.prox_left < (self.proximity_center - self.proximity_range_far) :
                self.state = "VEER_TOWARD_LEFT_WALL"

        elif self.state == "VEER_AWAY_FROM_LEFT_WALL":
#            if self.prox_left <= 30:
#                self.door_counter += 1
#                if self.door_counter == 4:
#                    self.state = "DOOR"
#                    self.old_state = "VEER_AWAY_FROM_LEFT_WALL"
            if time.ticks_diff(current_time, self.update_time["state_interval"]) > self.loop_time_ms:
                self.update_time["state_interval"] = current_time
                error = self.prox_left - self.proximity_center
                adjustment = min(abs(error) * 0.05, self.max_effort - self.nominal_effort)
                self.left_effort = self.nominal_effort + adjustment
                self.right_effort = self.nominal_effort - adjustment

                if self.distAhead < 10:
                    self.turn_angle = 90
                    self.turn_direction = "right"
                    self.state = "TURN"

            if self.prox_left <= (self.proximity_center + self.proximity_range_close):
                self.left_effort = self.nominal_effort
                self.right_effort = self.nominal_effort
                self.state = "FOLLOW_LEFT_WALL"

        elif self.state == "VEER_TOWARD_LEFT_WALL":
#            if self.prox_left <= 30:
#                self.door_counter += 1
#                if self.door_counter == 4:
#                    self.state = "DOOR"
#                    self.old_state = "VEER_TOWARD_LEFT_WALL"
#                    self.door_counter = 0
            if time.ticks_diff(current_time, self.update_time["state_interval"]) > self.loop_time_ms:
                self.update_time["state_interval"] = current_time
                self.left_effort = .4
                self.right_effort = .6
               
                if self.distAhead < 10:
                    self.turn_angle = 90
                    self.turn_direction = "right"
                    self.state = "TURN"
                   
            if self.prox_left >= (self.proximity_center - self.proximity_range_far):
                self.left_effort = self.nominal_effort
                self.right_effort = self.nominal_effort
                self.state = "FOLLOW_LEFT_WALL"
               
               
        elif self.state == "FOLLOW_RIGHT_WALL":
            self.right_effort = self.nominal_effort
            self.left_effort = self.nominal_effort
#            if self.prox_left <= 30:
#                self.door_counter += 1
#                if self.door_counter == 4:
#                    self.state = "DOOR"
#                    self.old_state = "FOLLOW_RIGHT_WALL"
#                    self.door_counter = 0
            if self.distAhead <= 10.0:
                self.turn_angle = 90
                self.turn_direction = "left"
                self.state = "TURN"
            elif self.prox_right > (self.proximity_center + self.proximity_range_close):
                self.state = "VEER_AWAY_FROM_RIGHT_WALL"

            elif self.prox_right < (self.proximity_center - self.proximity_range_far) :
                self.state = "VEER_TOWARD_RIGHT_WALL"

        elif self.state == "VEER_AWAY_FROM_RIGHT_WALL":
#            if self.prox_left <= 30:
#                self.door_counter += 1
#                if self.door_counter == 4:
#                    self.state = "DOOR"
#                    self.old_state = "VEER_AWAY_FROM_RIGHT_WALL"
#                    self.door_counter = 0
            if time.ticks_diff(current_time, self.update_time["state_interval"]) > self.loop_time_ms:
                self.update_time["state_interval"] = current_time
                error = self.prox_right - self.proximity_center
                adjustment = min(abs(error) * 0.05, self.max_effort - self.nominal_effort)
                self.left_effort = self.nominal_effort - adjustment
                self.right_effort = self.nominal_effort + adjustment

                if self.distAhead < 10:
                    self.turn_angle = 90
                    self.turn_direction = "left"
                    self.state = "TURN"

            if self.prox_right <= (self.proximity_center + self.proximity_range_close):
                self.left_effort = self.nominal_effort
                self.right_effort = self.nominal_effort
                self.state = "FOLLOW_RIGHT_WALL"

        elif self.state == "VEER_TOWARD_RIGHT_WALL":
#            if self.prox_left <= 30:
#                self.door_counter += 1
#                if self.door_counter == 4:
#                    self.state = "DOOR"
#                    self.old_state = "VEER_TOWARD_RIGHT_WALL"
            if time.ticks_diff(current_time, self.update_time["state_interval"]) > self.loop_time_ms:
                self.update_time["state_interval"] = current_time
                self.left_effort = .6
                self.right_effort = .4
               
                if self.distAhead < 10:
                    self.turn_angle = 90
                    self.turn_direction = "left"
                    self.state = "TURN"
                   
            if self.prox_right >= (self.proximity_center - self.proximity_range_far):
                self.left_effort = self.nominal_effort
                self.right_effort = self.nominal_effort
                self.state = "FOLLOW_RIGHT_WALL"
               
        elif self.state == "DOOR":
            if self.old_state == "FOLLOW_LEFT_WALL" or self.old_state == "VEER_TOWARD_LEFT_WALL" or self.old_state == "VEER_AWAY_FROM_LEFT_WALL":
                self.left_effort = self.nominal_effort
                self.right_effort = self.nominal_effort
                self.forward_dist_door = 1
                self.old_state = None
                distance_moved = 0
                self.turn_angle = 90
                self.turn_direction = "left"
                self.state = "TURN"
                self.door_counter = 0
            if self.old_state == "FOLLOW_RIGHT_WALL" or self.old_state == "VEER_TOWARD_RIGHT_WALL" or self.old_state == "VEER_AWAY_FROM_RIGHT_WALL":
                self.left_effort = self.nominal_effort
                self.right_effort = self.nominal_effort
                self.old_state = None
                self.turn_angle = 90
                self.turn_direction = "right"
                self.state = "TURN"
                self.door_counter = 0





        elif self.state == "TURN":
            if self.target_heading is None:
                self.target_heading = (self.heading + self.turn_angle) % 360
                #print(f"Turning {self.turn_direction} {self.turn_angle}to heading {self.target_heading}")

            heading_diff = abs(self.heading - self.target_heading)
            if heading_diff > 180:
                heading_diff = 360 - heading_diff

            elif heading_diff < 5:
                #print(f"Turn of {self.turn_angle} complete.")
                self.left_effort = 0
                self.right_effort = 0
                self.target_heading = None
                if self.prox_left > self.prox_right:
                    self.state = "FOLLOW_LEFT_WALL"
                elif self.prox_right > self.prox_left:
                    self.state = "FOLLOW_RIGHT_WALL"

            else:
                if self.turn_direction == "right":
                    self.left_effort = 0.6
                    self.right_effort = -0.6
                elif self.turn_direction == "left":
                    self.left_effort = -0.6
                    self.right_effort = 0.6
           

        self.print_state(self.state)
        drivetrain.set_effort(self.left_effort, self.right_effort)
        #print(self.distAhead)

sm = StateMachine()
board.led_on()

while True:
    sm.update_sensors()
    sm.evaluate_state()
	
