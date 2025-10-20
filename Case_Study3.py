import time
import random
import math

class Storage:
    """Represents VRM racks with aisles, total height 20 m, double-sided aisles."""

    def __init__(self, area_width=25, area_depth=30, slot_width=0.5, slot_depth=0.6, slot_height=0.5):
        self.area_width = area_width
        self.area_depth = area_depth
        self.slot_width = slot_width
        self.slot_depth = slot_depth
        self.slot_height = slot_height

        self.aisle_depth = 1.2 * slot_depth  # aisle size per your requirement
        self.base_clearance = 1.4 * slot_height  # clearance for robot movement
        self.usable_height = 20 - self.base_clearance
        self.levels = int(self.usable_height / slot_height)

        # Layout: 2 racks + 1 aisle repeated along depth, double sided storage
        self.unit_depth = (2 * slot_depth) + self.aisle_depth
        self.groups = int(self.area_depth / self.unit_depth)
        self.rack_columns = self.groups * 2  # racks count across depth
        self.slots_x = int(self.area_width / self.slot_width)

        self.total_slots = self.slots_x * self.rack_columns * self.levels
        self.slots_filled = 0

        # Track which slots are occupied: [rack][x][level]
        self.storage_map = [[[False for _ in range(self.levels)] for _ in range(self.slots_x)] for _ in range(self.rack_columns)]

    def store_box_random(self):
        """Store a box in a random free slot among all slots."""
        free_slots = []
        for rack in range(self.rack_columns):
            for x in range(self.slots_x):
                for level in range(self.levels):
                    if not self.storage_map[rack][x][level]:
                        free_slots.append((rack, x, level))
        if not free_slots:
            print("Storage is full.")
            return False

        rack, x, level = random.choice(free_slots)
        self.storage_map[rack][x][level] = True
        self.slots_filled += 1
        print(f"Stored box in Rack {rack+1}, X slot {x+1}, Level {level+1} ({self.slots_filled}/{self.total_slots})")
        return rack, x, level

    def get_slot_position(self, rack, x_slot, level):
        """Return physical position (x, y, z) in meters for a given slot."""
        x_pos = x_slot * self.slot_width
        y_pos = (rack // 2) * self.unit_depth + (self.slot_depth if (rack % 2) == 1 else 0)
        z_pos = level * self.slot_height
        return x_pos, y_pos, z_pos


class Robot:
    """Simulates a picking robot in the VRM layout."""

    def __init__(self, name, speed=3.0, climb_speed=0.8):
        self.name = name
        self.speed = speed
        self.climb_speed = climb_speed
        self.x_position = 0.0
        self.y_position = 0.0
        self.z_position = 0.0

    def move_horizontal(self, target_x, target_y):
        dist = ((target_x - self.x_position)**2 + (target_y - self.y_position)**2)**0.5
        duration = dist / self.speed if self.speed > 0 else 0
        print(f"{self.name} moving horizontally {dist:.2f} m in {duration:.2f}s...")
        time.sleep(0.1)
        self.x_position = target_x
        self.y_position = target_y
        return duration

    def move_vertical(self, target_z):
        distance = abs(target_z - self.z_position)
        duration = distance / self.climb_speed if self.climb_speed > 0 else 0
        direction = "up" if target_z > self.z_position else "down"
        print(f"{self.name} climbing {direction} by {distance:.2f} m ({duration:.2f}s)...")
        time.sleep(0.1)
        self.z_position = target_z
        return duration

    def pick_box(self):
        print(f"{self.name} picked a box.")
        time.sleep(0.05)

    def place_box(self):
        print(f"{self.name} placed a box.")
        time.sleep(0.05)


class Simulation:
    """Complete vertical robotic warehouse simulation."""

    def __init__(self, storage, robot, wait_time=5):
        self.storage = storage
        self.robot = robot
        self.wait_time = wait_time
        self.total_time = 0.0

    def run_cycle(self):
        """Performs single pick/store cycle with random slot pick."""

        result = self.storage.store_box_random()
        if not result:
            return False

        rack, x, level = result
        x_pos, y_pos, z_pos = self.storage.get_slot_position(rack, x, level)

        # Step 1: Move robot horizontally to rack position
        self.total_time += self.robot.move_horizontal(x_pos, y_pos)

        # Step 2: Climb vertically to storage height
        self.total_time += self.robot.move_vertical(z_pos)

        # Step 3: Pick box and descend
        self.robot.pick_box()
        self.total_time += self.robot.move_vertical(0)

        # Step 4: Move to delivery station (assume at X=25m, Y=0)
        self.total_time += self.robot.move_horizontal(25, 0)

        # Wait at delivery
        print(f"{self.robot.name} waiting {self.wait_time}s at delivery station...")
        time.sleep(0.1)
        self.total_time += self.wait_time

        # Step 5: Return to rack horizontal position
        self.total_time += self.robot.move_horizontal(x_pos, y_pos)

        # Step 6: Climb vertically to slot height again
        self.total_time += self.robot.move_vertical(z_pos)

        # Step 7: Place box and descend again
        self.robot.place_box()
        self.total_time += self.robot.move_vertical(0)

        return True

    def run(self):
        print(f"--- Starting {self.robot.name} VRM Simulation ---")
        for i in range(50):  # simulate 50 cycles as requested
            print(f"\nCycle {i+1}/50")
            if not self.run_cycle():
                print("Storage full. Ending simulation.")
                break

        print("\nSimulation complete.")
        print(f"Total simulated cycles: {self.storage.slots_filled}")
        print(f"Estimated total time: {self.total_time:.1f} seconds")
        avg_time = self.total_time / self.storage.slots_filled if self.storage.slots_filled else 0
        print(f"Average time per cycle: {avg_time:.2f} seconds")


if __name__ == "__main__":
    rack = Storage()
    robot = Robot(name="Custom_Robot", speed=3.0, climb_speed=0.8)
    sim = Simulation(storage=rack, robot=robot, wait_time=5)
    sim.run()
