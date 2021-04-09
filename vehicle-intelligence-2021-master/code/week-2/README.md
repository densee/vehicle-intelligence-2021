# Week 2 - Markov Localization

---

[//]: # (Image References)
## Assignment

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
    Initialize the position's probability  to zero.
    
    
Prediction : I calculate the probability of the vehicle moving to the current position from that prior (using pdf)
   
    for i in range(map_size):
    # calculate the probability in every position
    position_prob += norm_pdf(position-i, mov, stdev) * priors[i]


* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

Observation : I only calculated the probability for distance if the length of the observation data and the pseudo_range are the same, otherwise I set it to zero.

    if len(observations) == 0:
            distance_prob = 0.0
        elif len(observations) > len(pseudo_ranges):
            distance_prob = 0.0
        elif len(observations) < len(pseudo_ranges):
            distance_prob = 0.0
        else:
            for i in range(len(observations)):
                distance_prob *= norm_pdf(observations[i], pseudo_ranges[i], stdev)

The results of the experiment are attached as Markov_ysj.mp4.
 




