# -*- coding: cp949 -*-
import os
import pygame
import random
import math
import time
import matplotlib.pyplot as plt
import numpy as np  # 선형 회귀를 위해 추가
from datetime import datetime

# Fitts' Law Parameters
SCREEN_WIDTH = 1400  # Screen width
SCREEN_HEIGHT = 800  # Screen height
NUM_TRIALS = 11  # Number of targets to test (1 extra for excluding first trial)
HOLD_TIME = 0.5  # Time to hold cursor within target to register "click" in seconds

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Fitts' Law Usability Test")

# Set a clock to manage frame rate
clock = pygame.time.Clock()

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
LIGHTGRAY = (200, 200, 200)

# Font for text
font = pygame.font.Font(None, 36)  # Font size

# Fitts' Law Metrics
trials_data = []
start_time = None
hover_start_time = None
trial_count = 0
previous_target_x, previous_target_y = None, None

# Function to generate random target size and position
def generate_random_target(min_distance=50):
    radius = random.randint(10, 40)  # Random radius between 10 and 30 pixels
    while True:
        x = random.randint(radius, SCREEN_WIDTH - radius)
        y = random.randint(radius, SCREEN_HEIGHT - radius)
        # Check if new target is at least 'min_distance' away from the previous target
        if previous_target_x is None or previous_target_y is None or \
           (math.sqrt((x - previous_target_x) ** 2 + (y - previous_target_y) ** 2) >= min_distance):
            break
    return x, y, radius

# Calculate the time to move and click
def calculate_fitts_law(distance, target_width, time_taken):
    index_of_difficulty = math.log2((distance / target_width) + 1)
    movement_time = time_taken
    return index_of_difficulty, movement_time

# Calculate the angle between two points
def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    angle = math.degrees(math.atan2(dy, dx))
    return abs(angle)  # Return absolute value of angle

# Function to display multi-input prompt
def multi_input_prompt(prompt1, prompt2, prompt3):
    input_box1 = pygame.Rect(300, 200, 200, 50)  # Position and size for name
    input_box2 = pygame.Rect(300, 300, 200, 50)  # Position and size for device name
    input_box3 = pygame.Rect(300, 400, 200, 50)  # Position and size for number of trials
    color_inactive = pygame.Color('lightskyblue3')
    color_active = pygame.Color('dodgerblue2')
    color1, color2, color3 = color_inactive, color_inactive, color_inactive
    text1, text2, text3 = '', '', ''
    active1, active2, active3 = False, False, False
    clock = pygame.time.Clock()
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return None, None, None
            if event.type == pygame.MOUSEBUTTONDOWN:
                if input_box1.collidepoint(event.pos):
                    active1 = True
                    active2 = False
                    active3 = False
                elif input_box2.collidepoint(event.pos):
                    active1 = False
                    active2 = True
                    active3 = False
                elif input_box3.collidepoint(event.pos):
                    active1 = False
                    active2 = False
                    active3 = True
                else:
                    active1 = False
                    active2 = False
                    active3 = False
            if event.type == pygame.KEYDOWN:
                if active1:
                    if event.key == pygame.K_RETURN:
                        active1 = False
                    elif event.key == pygame.K_BACKSPACE:
                        text1 = text1[:-1]
                    else:
                        text1 += event.unicode
                elif active2:
                    if event.key == pygame.K_RETURN:
                        active2 = False
                    elif event.key == pygame.K_BACKSPACE:
                        text2 = text2[:-1]
                    else:
                        text2 += event.unicode
                elif active3:
                    if event.key == pygame.K_RETURN:
                        return text1, text2, text3  # Return the texts
                    elif event.key == pygame.K_BACKSPACE:
                        text3 = text3[:-1]
                    else:
                        text3 += event.unicode

        screen.fill(WHITE)
        
        # Render inputs
        txt_surface1 = font.render(text1, True, color1)
        txt_surface2 = font.render(text2, True, color2)
        txt_surface3 = font.render(text3, True, color3)
        width1 = max(200, txt_surface1.get_width()+10)
        width2 = max(200, txt_surface2.get_width()+10)
        width3 = max(200, txt_surface3.get_width()+10)
        input_box1.w = width1
        input_box2.w = width2
        input_box3.w = width3
        screen.blit(txt_surface1, (input_box1.x+5, input_box1.y+5))
        screen.blit(txt_surface2, (input_box2.x+5, input_box2.y+5))
        screen.blit(txt_surface3, (input_box3.x+5, input_box3.y+5))
        pygame.draw.rect(screen, color1, input_box1, 2)
        pygame.draw.rect(screen, color2, input_box2, 2)
        pygame.draw.rect(screen, color3, input_box3, 2)

        # Display prompts
        prompt_surface1 = font.render(prompt1, True, BLACK)
        prompt_surface2 = font.render(prompt2, True, BLACK)
        prompt_surface3 = font.render(prompt3, True, BLACK)
        screen.blit(prompt_surface1, (input_box1.x, input_box1.y - 30))
        screen.blit(prompt_surface2, (input_box2.x, input_box2.y - 30))
        screen.blit(prompt_surface3, (input_box3.x, input_box3.y - 30))

        pygame.display.flip()
        clock.tick(30)
        
# User input for name and device name
name, device_name, num_trials_input = multi_input_prompt("Please enter your name:", "Please enter your device name:", "Please enter number of trials (default 10):")
NUM_TRIALS = int(num_trials_input) if num_trials_input.isdigit() else 10
NUM_TRIALS += 1  # Increase for one extra trial to exclude the first

# Prepare filename based on user input and current date & time
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"{name}_{device_name}_{current_time}.txt"
filename = os.path.join(os.getcwd(), filename)

# Draw grid on the screen
def draw_grid():
    for x in range(0, SCREEN_WIDTH, 50):  # Vertical lines
        pygame.draw.line(screen, LIGHTGRAY, (x, 0), (x, SCREEN_HEIGHT))
    for y in range(0, SCREEN_HEIGHT, 50):  # Horizontal lines
        pygame.draw.line(screen, LIGHTGRAY, (0, y), (SCREEN_WIDTH, y))

# Main loop
running = True
current_target_x, current_target_y, current_radius = generate_random_target()

# Open file to save results
with open(filename, 'w') as f:
    f.write("Trial, D, W, Angle, ID, Time\n")  # Header for the CSV

    while running:
        screen.fill(WHITE)
        draw_grid()  # Draw the grid
        
        # Draw target with current radius
        pygame.draw.circle(screen, RED, (current_target_x, current_target_y), current_radius)
        
        # Get mouse position
        mouse_pos = pygame.mouse.get_pos()

        # Calculate distance from cursor to target center
        distance_to_target = math.sqrt((mouse_pos[0] - current_target_x) ** 2 + (mouse_pos[1] - current_target_y) ** 2)

        # If cursor is within target
        if distance_to_target <= current_radius:
            if hover_start_time is None:
                hover_start_time = time.time()
            elif time.time() - hover_start_time >= HOLD_TIME:
                # Cursor stayed inside the target for HOLD_TIME
                end_time = time.time()
                if start_time:
                    time_taken = end_time - start_time - HOLD_TIME
                    target_width = current_radius * 2  # Diameter of the target
                    distance_moved = math.sqrt((current_target_x - previous_target_x) ** 2 + (current_target_y - previous_target_y) ** 2) if previous_target_x is not None and previous_target_y is not None else 0
                    index_of_difficulty = math.log2((distance_moved / target_width) + 1) if distance_moved > 0 else 0
                    movement_time = time_taken
                    angle = calculate_angle(previous_target_x, previous_target_y, current_target_x, current_target_y) if previous_target_x is not None and previous_target_y is not None else 0
                    
                    # Calculate distance (D) and angle
                    if previous_target_x is not None and previous_target_y is not None:
                        distance_moved = math.sqrt((current_target_x - previous_target_x) ** 2 + (current_target_y - previous_target_y) ** 2)
                        angle = calculate_angle(previous_target_x, previous_target_y, current_target_x, current_target_y)
                    else:
                        distance_moved = 0
                        angle = 0

                    # Store trial data, but skip the first trial
                    if trial_count > 0:  # Exclude the first trial
                        trials_data.append({
                            "trial": trial_count,
                            "D": distance_moved,
                            "W": target_width,
                            "Angle": angle,
                            "ID": index_of_difficulty,
                            "time": movement_time
                        })
                        
                        # Write results to the file
                        f.write(f"{trial_count},{distance_moved:.2f},{target_width:.2f},{angle:.2f},{index_of_difficulty:.4f},{movement_time:.4f}\n")
                    
                    trial_count += 1
                    
                    if trial_count >= NUM_TRIALS:
                        running = False
                    else:
                        previous_target_x, previous_target_y = current_target_x, current_target_y
                        current_target_x, current_target_y, current_radius = generate_random_target()
                        start_time = time.time()
                        hover_start_time = None
                else:
                    start_time = time.time()
                    previous_target_x, previous_target_y = current_target_x, current_target_y
        else:
            hover_start_time = None  # Reset hover timer if cursor leaves target
        
        # Display trial info on screen
        trial_text = font.render(f'Trial: {min(trial_count, NUM_TRIALS - 1)}/{NUM_TRIALS - 1}', True, BLACK)
        screen.blit(trial_text, (10, 10))

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Limit the frame rate to 60 FPS
        clock.tick(60)
        pygame.display.flip()

# Test completed
pygame.quit()

# Print and plot Fitts' Law results (excluding first trial)
print("Fitts' Law Usability Test Results (First trial excluded):")
ids = []
times = []
for trial in trials_data:
    print(f"Trial {trial['trial']}: D = {trial['D']:.2f}, W = {trial['W']:.2f}, Angle = {trial['Angle']:.2f}, ID = {trial['ID']:.4f}, Time = {trial['time']:.4f}s")
    ids.append(trial['ID'])
    times.append(trial['time'])

""""
# Plot the results
plt.figure()
plt.scatter(ids, times, color='blue', label='Results')  # 데이터 포인트 그리기

# Linear regression
A = np.vstack([ids, np.ones(len(ids))]).T  # X, y 준비
m, b = np.linalg.lstsq(A, times, rcond=None)[0]  # 회귀선 계산
plt.plot(ids, m * np.array(ids) + b, 'r', label='Linear regression')  # 회귀선 그리기

plt.xlabel('Index of Difficulty (ID)')
plt.ylabel('Time (seconds)')
plt.title('Fitts\' Law Usability Test Results (First Trial Excluded)')
plt.grid(True)
plt.legend()
plt.show()
"""""

# Plot the results
plt.figure()
plt.scatter(ids, times, color='blue', label='Results')  # Plot data points

# Linear regression
A = np.vstack([ids, np.ones(len(ids))]).T  # Prepare X, y
m, b = np.linalg.lstsq(A, times, rcond=None)[0]  # Calculate regression line
plt.plot(ids, m * np.array(ids) + b, 'r', label='Linear regression')  # Plot regression line

plt.xlabel('Index of Difficulty (ID)')
plt.ylabel('Time (seconds)')
plt.title('Fitts\' Law Usability Test Results (First Trial Excluded)')
plt.grid(True)
plt.legend()

# Save the plot to a file before showing
plt.savefig(f"{name}_{device_name}_{current_time}_fitts_law_result.png")  # Save as PNG

# Show the plot
plt.show()

