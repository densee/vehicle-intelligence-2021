import matplotlib.pyplot as plt
import matplotlib.animation as animation
import imageio

from helper import GraphAnimator

from markov_localizer import initialize_priors
from markov_localizer import estimate_pseudo_range
from markov_localizer import motion_model
from markov_localizer import observation_model
from markov_localizer import normalize_distribution


filenames = []


if __name__ == '__main__':
    # Initialize graph data to an empty list
    graph = []
    # Std dev for initial position
    position_stdev = 1.0
    # Std dev for control (movement)
    control_stdev = 1.0
    # Std dev for observation (measurement)
    observation_stdev = 1.0
    # Assumed constant velocity of vehicle
    mov_per_timestep = 1.0

    # Size of the map
    map_size = 25
    # Map (landmark positions)
    landmark_positions = [3, 9, 14, 23]

    # Observation data
    observations = [
        [1, 7, 12, 21],     #x_t : 1
        [0, 6, 11, 20],     #x_t : 2
        [5, 10, 19],        #x_t : 3
        [4, 9, 18],         #x_t : 4
        [3, 8, 17],         #x_t : 5
        [2, 7, 16],         #x_t : 6
        [1, 6, 15],         #x_t : 7
        [0, 5, 14],         #x_t : 8
        [4, 13],            #x_t : 9
        [3, 12],            #x_t : 10
        [2, 11],            #x_t : 11
        [1, 10],            #x_t : 12
        [0, 9],             #x_t : 13
        [8],                #x_t : 14
        [7],                #x_t : 15
        [6],                #x_t : 16
        [5],                #x_t : 17
        [4],                #x_t : 18
        [3],                #x_t : 19
        [2],                #x_t : 20
        [1],                #x_t : 21
        [0],                #x_t : 22
        [],                 #x_t : 23
        [],                 #x_t : 24
        [],                 #x_t : 25
    ]

    # Initialize priors (initial belief)
    priors = initialize_priors(
        map_size, landmark_positions, position_stdev
    )

    # Cycle through timesteps
    for t in range(len(observations)):

        print("---------------TIME STEP---------------")
        print("t = %d" % t)
        print("-----Motion----------OBS----------------PRODUCT--")

        posteriors = [0.0] * map_size
        # Step through each pseudo position p (to determine pdf)
        for pseudo_position in range(map_size):
            # Prediction:
            # Calculate probability of the vehicle being at position p

            #print("pseudo_position = %d" % pseudo_position)

            motion_prob = motion_model(
                pseudo_position, mov_per_timestep, priors,
                map_size, control_stdev
            )
            # Get pseudo range
            pseudo_ranges = estimate_pseudo_range(
                landmark_positions, pseudo_position
            )

            #for observation in observations[t]:
            #    print("observation = %f" % observation)

            #for pseudo_range in pseudo_ranges:
            #    print("pseudo_range = %f" % pseudo_range)

            #for landmark_position in landmark_positions:
            #    print("landmark_position = %f" % landmark_position)

            # Measurement update:
            # Calculate observation probability
            observation_prob = observation_model(
                landmark_positions, observations[t],
                pseudo_ranges, observation_stdev
            )

            # Calculate posterior probability
            posteriors[pseudo_position] = motion_prob * observation_prob


            print("%f\t%f\t%f" % (motion_prob,
                                  observation_prob,
                                  posteriors[pseudo_position]))

            #print("pseudo_position = %f" % pseudo_position)


        # Normalize the posterior probability distribution
        posteriors = normalize_distribution(posteriors)

        # Update priors with posteriors
        priors = posteriors

        #for i in range(map_size):
        #    print("i = %d" % i)
        #    print("priors = %f" % priors[i])

        # Collect data to plot according to timestep.
        graph.append(posteriors)

    # Now we generate an animated plot with the saved data.
    fig, ax = plt.subplots(figsize=(8, 8), num='Markov Localization')
    bgraph = plt.bar(range(map_size), [0] * map_size)
    plt.ylim(0, 1)
    graph_animator = GraphAnimator(bgraph, graph)
    ani = animation.FuncAnimation(
        fig, graph_animator.animate, blit=True, interval=2500, repeat=True,
        frames=len(graph)
    )

    plt.show()
