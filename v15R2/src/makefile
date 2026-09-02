# A3_PATCH_31_INCREMENTAL_MAKEFILE
CC = gcc
# A3_PATCH_32_PORTABLE_GRID_RESTORE
ENABLE_NATIVE ?= 0
NATIVE_FLAG := $(if $(filter 1,$(ENABLE_NATIVE)),-march=native,)
CFLAGS = $(shell pkg-config --cflags gtk+-3.0 epoxy) -I. -O3 $(NATIVE_FLAG) -Wall -Wextra -MMD -MP
LIBS = $(shell pkg-config --libs gtk+-3.0 epoxy) -lm

BUILD_DIR := build
SOURCES := $(shell find . -name '*.c' | sed 's|^\./||')
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SOURCES))
DEPS := $(OBJECTS:.o=.d)

engine: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(LIBS) -o engine

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) engine test_*

.PHONY: clean

-include $(DEPS)

# MPE_FTC_059D: headless two-world independence test
TWO_WORLD_SOURCES := tests/two_world_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	config/mpe_config.c config/mpe_config_schema.c
test_two_world: $(TWO_WORLD_SOURCES)
	$(CC) $(CFLAGS) -DMPE_TWO_WORLD_TEST $(TWO_WORLD_SOURCES) -lm -o test_two_world
	./test_two_world

# MPE_FTC_068: headless revolute pendulum test
REVOLUTE_TEST_SOURCES := tests/revolute_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	config/mpe_config.c config/mpe_config_schema.c
test_revolute: $(REVOLUTE_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_REVOLUTE_TEST $(REVOLUTE_TEST_SOURCES) -lm -o test_revolute
	./test_revolute

# MPE_FTC_077: headless teleop drive test
TELEOP_TEST_SOURCES := tests/teleop_drive_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c
test_teleop_drive: $(TELEOP_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_TELEOP_DRIVE_TEST $(TELEOP_TEST_SOURCES) -lm -o test_teleop_drive
	./test_teleop_drive

# MPE_FTC_078: ASan+UBSan teleop drive test for heap-corruption diagnosis
test_teleop_drive_asan: $(TELEOP_TEST_SOURCES)
	$(CC) $(CFLAGS) -O1 -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer \
	  -DMPE_TELEOP_DRIVE_TEST $(TELEOP_TEST_SOURCES) -lm \
	  -fsanitize=address -fsanitize=undefined -o test_teleop_drive_asan
	./test_teleop_drive_asan

# MPE_FTC_080: headless mecanum drive test
MECANUM_TEST_SOURCES := tests/mecanum_drive_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c
test_mecanum_drive: $(MECANUM_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_MECANUM_DRIVE_TEST $(MECANUM_TEST_SOURCES) -lm -o test_mecanum_drive
	./test_mecanum_drive

# MPE_FTC_092: cylinder drop test
CYLINDER_DROP_TEST_SOURCES = tests/cylinder_drop_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_cylinder_drop: $(CYLINDER_DROP_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_CYLINDER_DROP_TEST $(CYLINDER_DROP_TEST_SOURCES) -lm -o test_cylinder_drop
	./test_cylinder_drop

# MPE_FTC_093a: driven-wheel propulsion proof
DRIVEN_WHEEL_TEST_SOURCES = tests/driven_wheel_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_driven_wheel: $(DRIVEN_WHEEL_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_DRIVEN_WHEEL_TEST $(DRIVEN_WHEEL_TEST_SOURCES) -lm -o test_driven_wheel
	./test_driven_wheel

# MPE_FTC_093e: math3_inverse diagnostic
MATH3_INV_TEST_SOURCES = tests/math3_inverse_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_math3_inverse: $(MATH3_INV_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_MATH3_INVERSE_TEST $(MATH3_INV_TEST_SOURCES) -lm -o test_math3_inverse
	./test_math3_inverse

# MFS_FTC_INTEGRATION: full FTC robot integration test
FTC_INTEGRATION_SOURCES := tests/ftc_integration_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c

test_ftc_integration: $(FTC_INTEGRATION_SOURCES)
	$(CC) $(CFLAGS) -DMPE_FTC_INTEGRATION_TEST $(FTC_INTEGRATION_SOURCES) -lm -o test_ftc_integration
	./test_ftc_integration

# MFS_FTC_DEBUG: diagnostic test
FTC_DEBUG_SOURCES := tests/ftc_debug_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c

test_ftc_debug: $(FTC_DEBUG_SOURCES)
	$(CC) $(CFLAGS) -DMPE_FTC_DEBUG_TEST $(FTC_DEBUG_SOURCES) -lm -o test_ftc_debug
	./test_ftc_debug
