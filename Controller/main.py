import serial
import re
import pygame
import sys
import pyautogui

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 9600

# Regex to parse lines like: x:  18, y:  32
line_pattern = re.compile(r'\s*x:\s*(-?\d+),\s*y:\s*(-?\d+)')

WIDTH, HEIGHT = 400, 400
JOYSTICK_RADIUS = 150
KNOB_RADIUS = 20
CENTER = (WIDTH // 2, HEIGHT // 2)

MOUSE_SENSITIVITY = 5  # Adjust this for mouse speed
MOUSE_MOVE_THRESHOLD = 0.5  # Minimum move (pixels) to send mouse event
DEADZONE = 8  # Joystick deadzone around zero

def map_value(val, from_min, from_max, to_min, to_max):
    return int((val - from_min) / (from_max - from_min) * (to_max - to_min) + to_min)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Joystick to Mouse")
    font = pygame.font.SysFont(None, 24)
    clock = pygame.time.Clock()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0)  # Non-blocking read
        print(f"Opened serial port {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Failed to open serial port {SERIAL_PORT}: {e}")
        return

    x_val = 0
    y_val = 0

    running = True
    while running:
        # Process Pygame events to allow clean exit
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Read all available lines from serial port (non-blocking)
        while True:
            try:
                raw_line = ser.readline()
            except serial.SerialException:
                print("Serial exception during readline.")
                break

            if not raw_line:
                break  # No more data

            try:
                line = raw_line.decode('utf-8').strip()
            except UnicodeDecodeError:
                continue  # Skip malformed lines

            match = line_pattern.match(line)
            if match:
                x_val = int(match.group(1))
                y_val = int(match.group(2))

        # Calculate relative mouse movement from joystick values
        move_x = 0
        move_y = 0

        if abs(x_val) > DEADZONE:
            move_x = -(x_val / 128) * MOUSE_SENSITIVITY * 10  # Invert X axis
        if abs(y_val) > DEADZONE:
            move_y = -(y_val / 128) * MOUSE_SENSITIVITY * 10  # Invert Y axis for screen coords

        # Only move mouse if movement exceeds threshold (to avoid jitter)
        if abs(move_x) > MOUSE_MOVE_THRESHOLD or abs(move_y) > MOUSE_MOVE_THRESHOLD:
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
        clock.tick(60)  # 60 FPS cap

    ser.close()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
