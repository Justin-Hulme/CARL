import serial
import re
import pygame
import sys
import pyautogui
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 9600

line_pattern = re.compile(r'\r?\s*x:\s*(-?\d+), y:\s*(-?\d+)')

WIDTH, HEIGHT = 400, 400
JOYSTICK_RADIUS = 150
KNOB_RADIUS = 20
CENTER = (WIDTH // 2, HEIGHT // 2)

# Sensitivity for mouse movement
MOUSE_SENSITIVITY = 1.5  # tweak this for faster/slower cursor

def map_value(val, from_min, from_max, to_min, to_max):
    return int((val - from_min) / (from_max - from_min) * (to_max - to_min) + to_min)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Joystick Visualizer")
    font = pygame.font.SysFont(None, 24)
    clock = pygame.time.Clock()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        print(f"Opened serial port {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        return

    x_val = 0
    y_val = 0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        lines_read = 0
        while lines_read < 5:
            try:
                raw_line = ser.readline()
            except serial.SerialException:
                print("Serial exception during readline.")
                break

            if not raw_line:
                break

            try:
                line = raw_line.decode('utf-8').strip()
            except UnicodeDecodeError:
                continue

            match = line_pattern.match(line)
            if match:
                x_val = int(match.group(1))
                y_val = int(match.group(2))
            lines_read += 1

        # Map joystick values (-128 to 127) to relative mouse movement
        # Smaller joystick movement = smaller mouse move; deadzone around 0 to avoid drift
        deadzone = 8

        move_x = 0
        move_y = 0

        if abs(x_val) > deadzone:
            move_x = -(x_val / 128) * MOUSE_SENSITIVITY * 10  # scale movement
        if abs(y_val) > deadzone:
            move_y = -(y_val / 128) * MOUSE_SENSITIVITY * 10  # invert Y axis for screen coords

        if move_x != 0 or move_y != 0:
            pyautogui.moveRel(move_x, move_y)

        # Draw joystick visualization
        x_pos = map_value(x_val, -128, 127, -JOYSTICK_RADIUS, JOYSTICK_RADIUS)
        y_pos = map_value(y_val, -128, 127, -JOYSTICK_RADIUS, JOYSTICK_RADIUS)

        screen.fill((30, 30, 30))
        pygame.draw.circle(screen, (100, 100, 100), CENTER, JOYSTICK_RADIUS, 5)
        knob_pos = (CENTER[0] + x_pos, CENTER[1] - y_pos)
        pygame.draw.circle(screen, (0, 200, 0), knob_pos, KNOB_RADIUS)

        text_raw = font.render(f"Raw x: {x_val}  y: {y_val}", True, (255, 255, 255))
        screen.blit(text_raw, (10, 10))

        pygame.display.flip()
        clock.tick(60)

    ser.close()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
