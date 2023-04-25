# Random Control Matrix
Code for the paper, "Chance Constrained Stochastic Optimal Control for Linear Systems with a Time Varying Random Control Matrix," Under Review, CCTA 2023.

arXiv: [https://arxiv.org/abs/2302.01863](https://arxiv.org/abs/2302.01863)

## Requirements
* CVX [http://cvxr.com/cvx/](http://cvxr.com/cvx/)

## Examples
### Impulse Control Inaccuracies
We use a 6d CWH system with Gamma random variables in the control matrix to model the dynamics of a single-satellite rendezvous operation with impulsive control inaccuracies. This example compares the efficacy of the proposed approach with with its predisessor based on Cantelli's inequality.

### Under-performing Actuators
We use a 6d CWH system with Beta random variables in the control matrix to model the dynamics of a single-satellite rendezvous operation to model under performing actuators. This example compares the efficiency of the proposed approach with that of the scenario approach.
