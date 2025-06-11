# vehicle-route-optimizer

Creating a full Python program for a "vehicle-route-optimizer" involves a number of steps and components, each of which can be quite complex depending on the specific requirements and constraints of your problem. Below is a simplified example using Google OR-Tools, which is a powerful library for solving optimization problems, such as vehicle routing.

Before you run the code, make sure to install the required libraries:

```bash
pip install ortools
```

Here's a basic Python program implementing a vehicle-route-optimizer:

```python
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import sys

def create_data_model():
    """Stores the data for the problem."""
    # Example distance matrix representing distances between locations
    data = {}
    data['distance_matrix'] = [
        [0, 2, 9, 10],
        [1, 0, 6, 4],
        [15, 7, 0, 8],
        [6, 3, 12, 0],
    ]
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def distance_callback(from_index, to_index, data):
    """Returns the distance between the two nodes."""
    return data['distance_matrix'][from_index][to_index]

def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {0} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {0}\n'.format(manager.IndexToNode(index))
    plan_output += 'Distance of the route: {}m\n'.format(route_distance)
    print(plan_output)

def main():
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Register distance callback
    transit_callback_index = routing.RegisterTransitCallback(
        lambda from_index, to_index: distance_callback(from_index, to_index, data))

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Return solution
    if solution:
        print_solution(manager, routing, solution)
    else:
        print('No solution found!')

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"An error occurred: {e}", file=sys.stderr)
```

### Explanation:

1. **Data Model**: The `create_data_model()` function defines the problem data, such as the distance matrix, the number of vehicles, and the depot (starting point).

2. **Distance Callback**: The `distance_callback` function returns the distance between two nodes.

3. **Routing Manager and Model**: Uses `RoutingIndexManager` and `RoutingModel` to handle the routing logic.

4. **Cost Evaluator**: Registers a callback to compute travel costs between nodes.

5. **Solver Parameters**: Configures the solution strategy.

6. **Solution Output**: Once a solution is found, it prints the vehicle's route and the total distance.

7. **Error Handling**: The `try` block in the `main()` function catches and prints any exceptions.

This basic model can be expanded to consider constraints like vehicle capacities, delivery windows, or different cost structures. Implementing such features would require an in-depth understanding of optimization algorithms and possibly more advanced libraries or even custom algorithms, depending on the complexity and scale of the problem.