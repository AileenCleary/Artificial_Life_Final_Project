import subprocess
import random
from collections import Counter
from rigid_body import load_results
from concurrent.futures import ProcessPoolExecutor
import uuid
import sys
import matplotlib.pyplot as plt
from datetime import datetime
import os
import numpy as np
import ast
import re

mutation_rate = 0.1
visualize = False
save_results = True
# population_size =10
# generations = 10
# path = "sine"
# shape = "wheel"
# iters = 20

def evaluate_individual(n_boxes):
    """
    Run gradient-descent optimization (rigid_body.py) on a rigid-body of n_boxes.

    Inputs:
        n_boxes: Number of boxes (objects) for rigid-body. See build_robot_skeleton() for more.

    Outputs:
        n_boxes: Number of objects for a given rigid-body.
        final_loss: Loss value for the final iteration of optimization.
        losses: List of all losses during optimization.
    
    """
    
    run = sys.argv[1]
    shape = sys.argv[2]
    iters = int(sys.argv[3])
    path = sys.argv[4]
    
    print(f"Testing num_boxes = {n_boxes}")

    unique_id = uuid.uuid4().hex  # Generate a unique ID
    filename = f"{n_boxes}_evo_results_{unique_id}.pkl"

    result = subprocess.run(
        ["python", "rigid_body.py", str(n_boxes), run, shape, str(iters), path, filename],
        capture_output=True, text=True
    )
    # subprocess.run("ti cache clean -p C:/taichi_cache/ticache")
    if result.returncode == 0:
        robot_id, losses = load_results(directory='results', filename=filename)
        final_loss = losses[-1]
        print(f"Robot ID: {robot_id}, Losses: {losses}")
        return n_boxes, final_loss, losses
    else:
        print("Error running rigid_body.py:", result.stderr)
        return n_boxes, float('inf'), []  # Assign a high loss for failed runs

def parallel_evolutionary_optimize():
    """
    Run an evolution-based optimization simulation. Each individual in a population tested in parallel.
    New population created through crossover and mutation.

    Parameters:
        population_size: Number of individuals per population.
        generations: Number of generations to run.
        mutation_rate: Rate of mutation.
        iters: Number of iterations of optimization to run per individual.
        shape: Shape of rigid-bodies in a simulation, either "wheel" or "circle"
        path: Desired path for rigid bodies to follow. 

    Output:
        best_num_boxes: Number of boxes from the fittest rigid-body.
        all_generation_losses: List of all losses across all generations.
        all_populations: List of all populations across all generations.
        mode_num_boxes: Most populous num_boxes configuration in the final generation.
    """
    num_boxes_range = range(4, 13)
    population = [random.choice(num_boxes_range) for _ in range(population_size)]
    
    all_generation_losses = []
    all_populations = []

    for generation in range(generations):
        print(f"Generation {generation + 1}/{generations}")
        fitness_scores = []
        generation_losses = []

        # Use ProcessPoolExecutor for parallel execution
        with ProcessPoolExecutor() as executor:
            results = list(executor.map(evaluate_individual, population))

        for n_boxes, final_loss, losses in results:
            if losses:
                fitness_scores.append((n_boxes, final_loss))
                generation_losses.append(losses)
                print(f"Final loss for num_boxes = {n_boxes}: {final_loss}")

        all_generation_losses.append(generation_losses)
        all_populations.append(population.copy())

        # Select the best individuals
        fitness_scores.sort(key=lambda x: x[1])
        selected = fitness_scores[:population_size // 2]

        # Crossover and Mutation
        new_population = []
        for _ in range(population_size):
            parent1 = random.choice(selected)[0]
            parent2 = random.choice(selected)[0]
            child = (parent1 + parent2) // 2
            if random.random() < mutation_rate:
                child = random.choice(num_boxes_range)
            new_population.append(child)

        population = new_population
        #subprocess.run("ti cache clean -p C:/taichi_cache/ticache")

    final_population_counts = Counter(population)
    mode_num_boxes = final_population_counts.most_common(1)[0][0]

    best_num_boxes = min(fitness_scores, key=lambda x: x[1])[0]
    print(f"Best num_boxes: {best_num_boxes} with loss: {min(fitness_scores, key=lambda x: x[1])[1]}")

    return best_num_boxes, all_generation_losses, all_populations, mode_num_boxes

def save_generation_losses(all_generation_losses, directory="results", filename=None):
    """
    Saves collected generation losses for a given run. By default, saves to directory "results", creates directory
    if not there, and saves the .txt file with a unique name according to simulation parameters and time.

    Inputs:
        all_generation_losses: Collected list of generation losses.
        directory: Name of directory to save to.
        filename: Name to save the file as.
    """
    os.makedirs(directory, exist_ok=True)
    if filename is None:
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename = f'generation_losses_{path}_{shape}_{timestamp}.txt'
    filepath = os.path.join(directory, filename)
    with open(filepath, 'w') as f:
        for generation_losses in all_generation_losses:
            for losses in generation_losses:
                f.write(','.join(map(str, losses)) + '\n')
            f.write('\n')
    print(f"Saved generation losses to {filepath}")

def plot_generation_losses(all_generation_losses, all_populations, directory="results"):
    """
    Plots the average loss value per iteration of each num_boxes value across all generations. By default, saves to directory "results", creates directory
    if not there, and saves the .png file with a unique name according to simulation parameters and time. 
    If visualize=true (false by default), will open graph.

    Inputs:
        all_generation_losses: Collected list of generation losses.
        all_populations: Collected list of populations.
        directory: Name of directory to save to.
    """
    os.makedirs(directory, exist_ok=True)
    num_boxes_losses = {}
    
    # Group losses by num_boxes
    for generation_losses, population in zip(all_generation_losses, all_populations):
        for num_boxes, losses in zip(population, generation_losses):
            if num_boxes not in num_boxes_losses:
                num_boxes_losses[num_boxes] = []
            num_boxes_losses[num_boxes].append(losses)
    
    # Plot average loss per generation for each num_boxes
    for num_boxes, losses in num_boxes_losses.items():
        avg_losses = [sum(loss) / len(loss) for loss in zip(*losses)]
        plt.plot(avg_losses, label=f'num_boxes {num_boxes}')

    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    filename = f'generation_losses_plot_{path}_{shape}_{timestamp}.png'
    filepath = os.path.join(directory, filename)
    plt.figtext(0.0, 0.0, f'Iterations: {iters}, Shape: {shape}, Population Size: {population_size}, Generations: {generations}')
    plt.xlabel('Iteration')
    plt.ylabel('Average Loss')
    plt.title('Average Loss per Iteration for Each num_boxes')
    plt.legend()
    plt.grid(True)
    if save_results:
        plt.savefig(filepath)
    if visualize:
        plt.show(block=False)

def plot_population_distribution_and_avg_loss(all_generation_losses, all_populations, directory="results"):
    """
    Plots the population distribution (num_boxes value) and average loss per generation. By default, saves to directory "results", creates directory
    if not there, and saves the .png file with a unique name according to simulation parameters and time. 
    If visualize=true (false by default), will open graph.

    Inputs:
        all_generation_losses: Collected list of generation losses.
        all_populations: Collected list of populations.
        directory: Name of directory to save to.
    """
    generations = range(len(all_populations))
    os.makedirs(directory, exist_ok=True)
    
    num_boxes_counts = {}
    avg_losses_per_generation = []

    for gen_idx, (generation_losses, population) in enumerate(zip(all_generation_losses, all_populations)):
        avg_loss = np.mean([loss[-1] for loss in generation_losses])  
        avg_losses_per_generation.append(avg_loss)
        
        for num_boxes in population:
            if num_boxes not in num_boxes_counts:
                num_boxes_counts[num_boxes] = [0] * len(all_populations)
            num_boxes_counts[num_boxes][gen_idx] += 1 
    
    # Plot 1: Population distribution per generation
    plt.figure(figsize=(12, 5))
    for num_boxes, counts in num_boxes_counts.items():
        plt.plot(generations, counts, marker='o', label=f'num_boxes {num_boxes}')
    plt.figtext(0.0, 0.0, f'Iterations: {iters}, Shape: {shape}, Population Size: {population_size}, Generations: {generations}')
    plt.xlabel('Generation')
    plt.ylabel('Count of num_boxes in Population')
    plt.title('Population Distribution Over Generations')
    plt.legend()
    plt.grid(True)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    filename = f'population_distribution_plot_{path}_{shape}_{timestamp}.png'
    filepath = os.path.join(directory, filename)
    if save_results:
        plt.savefig(filepath)
    if visualize:
        plt.show(blocking=False)
    
    # Plot 2: Average loss per generation
    plt.figure(figsize=(12, 5))
    plt.plot(generations, avg_losses_per_generation, marker='o', color='red', label='Avg Loss')
    plt.figtext(0.0, 0.0, f'Iterations: {iters}, Shape: {shape}, Population Size: {population_size}, Generations: {generations}')
    plt.xlabel('Generation')
    plt.ylabel('Average Loss')
    plt.title('Average Population Loss Per Generation')
    plt.legend()
    plt.grid(True)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    filename = f'avg_loss_generation_plot_{path}_{shape}_{timestamp}.png'
    filepath = os.path.join(directory, filename)
    if save_results:
        plt.savefig(filepath)
    if visualize:
        plt.show(blocking=False)

def main():
    # best_num_boxes, all_generation_losses, all_populations, mode_boxes = alt_evolutionary_optimize("wheel", 20, "sin_1")
    # #save_generation_losses(all_generation_losses, 'generation_losses.txt')
    # plot_generation_losses(all_generation_losses, all_populations)
    # plot_population_distribution_and_avg_loss(all_generation_losses, all_populations)
    
    if run == "para":
        best_num_boxes, all_generation_losses, all_populations, mode_boxes = parallel_evolutionary_optimize()
        if save_results:
            save_generation_losses(all_generation_losses)
        plot_generation_losses(all_generation_losses, all_populations)
        plot_population_distribution_and_avg_loss(all_generation_losses, all_populations)
        #result = subprocess.run(["python", "rigid_body.py", str(mode_boxes), "part_two"], capture_output=True, text=True)
    
if __name__ == '__main__':
    import sys
    if len(sys.argv) != 7: 
        print(
        "Usage: python3 main_opt.py [run=para] [shape=wheel/circle] [iter=1, 2, ..., n] [path=cos/sin/parabola] [generations=1, 2, ..., n] [population_size=1, 2, ..., n]"
        )   
        exit(-1)
    else: 
        generations = int(sys.argv[5])
        population_size = int(sys.argv[6])
        run = sys.argv[1]
        shape = sys.argv[2]
        iters = int(sys.argv[3])
        path = sys.argv[4]
        print(sys.argv)
    main()