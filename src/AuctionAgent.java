

//the list of imports
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import logist.Measures;
import logist.behavior.AuctionBehavior;
import logist.agent.Agent;
import logist.simulation.Vehicle;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * A very simple auction agent that assigns all tasks to its first vehicle and
 * handles them sequentially.
 * 
 */
@SuppressWarnings("unused")
public class AuctionAgent implements AuctionBehavior {

	private Topology topology;
	private TaskDistribution distribution;
	private Agent agent;
	//private Random random;
	private List<Vehicle> vehicles;
	private List<Task> tasks;
	private Solution currentPlans;
	private Solution futurePlans;
	//private City currentCity;

	@Override
	public void setup(Topology topology, TaskDistribution distribution,
			Agent agent) {

		this.topology = topology;
		this.distribution = distribution;
		this.agent = agent;
		this.vehicles = agent.vehicles();
		this.tasks = new ArrayList<Task>();
		
		Solution.vehicles = this.vehicles;
		
		Solution s = new Solution();
		s.cost = 0.0;
		currentPlans = s;
		futurePlans = s;
	
		//this.currentCity = vehicle.homeCity();

		/*long seed = -9019554669489983951L * currentCity.hashCode() * agent.id();
		this.random = new Random(seed);*/
	}

	@Override
	public void auctionResult(Task previous, int winner, Long[] bids) {
		if (winner == agent.id()) {
			System.out.println(previous.reward);
			this.tasks.add(previous);
			this.futurePlans.tasks = this.tasks;
			this.currentPlans = this.futurePlans;
		}
	}
	
	@Override
	public Long askPrice(Task task) {

		//TODO compute marginal cost
		ArrayList<Task> futureTasks = new ArrayList<Task>(tasks);
		futureTasks.add(task);
		
		//TODO Loop
		this.futurePlans = centralizedPlan(vehicles, futureTasks, 0.8);
		
		
		//TODO never bid negative
		double marginalCost =futurePlans.cost - currentPlans.cost;

		return (long) Math.round(marginalCost+1);
	}
	
	@Override
	public List<Plan> plan(List<Vehicle> vehicles, TaskSet tasks) {
		System.out.println(tasks);
		System.out.println(currentPlans.tasks);
		//TODO pourquoi ça marche pas
		//return currentPlans.getPlan();
		return centralizedPlan(vehicles, new ArrayList<Task>(tasks), 0.8).getPlan();
	}

	private Solution centralizedPlan(List<Vehicle> vehicles, ArrayList<Task> tasks, double p) {

		Solution.vehicles = vehicles;
		Solution.tasks = tasks;
		Solution Aold = new Solution();
		Aold.cost = Double.POSITIVE_INFINITY;
		
		//The biggest vehicle handles tasks sequentially
		Solution A = selectInitialSolution(vehicles, tasks);
		
		if (!A.verifyConstraints()) {
			System.err.println("At least one task is too big for the biggest capacity's vehicle!");
			System.exit(-1);
		}
		
		List<Solution> N = null;

		int count = 0;
		
		//We continue while we improve
		while (count < 10000 && A.cost < Aold.cost) {
			Aold = new Solution(A);
			
			N = chooseNeighbours(Aold, tasks, vehicles);
			
			//We also add the old state in order to prevent NullPointerExceptions if no neighbour is better
			N.add(Aold);
			
			//Select the best solution among the neighbours (and the current solution)
			if(Math.random() < p){
			A = localChoice(N);
			} else {
				A = N.get((int) Math.random()*N.size());
			}
			
			//System.out.println("[Info] Iter " + count + " : " + A.cost);
			count++;
		}
		
		return A;
	

	}

	/**
	 * As an initial solution, we just take the vehicle with biggest capacity
	 * and assign all the tasks to it, sequentially.
	 * @param vehicles the list of vehicles
	 * @param tasks the liste of tasks
	 * @return an initial solution
	 */
	private Solution selectInitialSolution(List<Vehicle> vehicles, List<Task> tasks) {

		Vehicle biggestVehicle = null;
		int maxCapacity = 0;
		for (Vehicle vehicle : vehicles) {
			if (vehicle.capacity() > maxCapacity) {
				maxCapacity = vehicle.capacity();
				biggestVehicle = vehicle;
			}
		}

		Solution initialSolution = new Solution();

		for (Task task : tasks) {
			initialSolution.actionsList.get(biggestVehicle).add(new Action(task, "pickup"));
			initialSolution.actionsList.get(biggestVehicle).add(new Action(task, "delivery"));
		}

		initialSolution.computeCost();

		return initialSolution;
		
	}

	private List<Solution> chooseNeighbours(Solution Aold, List<Task> tasks, List<Vehicle> vehicles) {

		List<Solution> N = new ArrayList<Solution>();
		
		for (Vehicle vi : vehicles) {
			
			if (!Aold.actionsList.get(vi).isEmpty()) {
				
				// Applying the changing vehicle operation:
				for (Vehicle vj : vehicles) {
					if (!vj.equals(vi)) {
						List<Solution> As = changingVehicle(Aold, vi, vj);
						N.addAll(As);
					}
				}
				
				// Applying the changing task order operation:
				List<Solution> As = changingTaskOrder(Aold, vi);
				N.addAll(As);
						
			}
			
		}

		return N;

	}

	/*
	 * We choose the best local solution. If multiple solutions are equally good, we choose one at random.
	 */
	private Solution localChoice(List<Solution> N) {

		List<Solution> bestSolutions = new ArrayList<Solution>();
		double leastCost = Double.POSITIVE_INFINITY;

		for (Solution solution : N) {
			
			if (solution.cost < leastCost) {
				leastCost = solution.cost;
				bestSolutions = new ArrayList<Solution>();
				bestSolutions.add(solution);
				
			} else if (solution.cost == leastCost) {
				bestSolutions.add(solution);
			}
		}
	
		return bestSolutions.get((int) (Math.random() * bestSolutions.size()));
		
	}
	
	/* 
	 * We generate all the neighbours by giving one task handled by v1 and giving it to v2
	 */
	
	public List<Solution> changingVehicle(Solution A, Vehicle v1, Vehicle v2) {
		
		List<Solution> solutions = new ArrayList<Solution>();
		
		//We can give any task of v1 to v2
		for(int actionIndex = 0; actionIndex < A.actionsList.get(v1).size(); actionIndex++) {
			
			Solution A1 = new Solution(A);
			
			Action pickupAction = A.actionsList.get(v1).get(actionIndex);
			
			if (pickupAction.actionType.equals("pickup")) { // a pickup action
				
				Action deliveryAction = new Action(pickupAction.task, "delivery");
				
				// We remove the actions from v1
				A1.actionsList.get(v1).remove(pickupAction);
				A1.actionsList.get(v1).remove(deliveryAction);
				
				// And then put them anywhere in the actionsList of v2
				for (int i = 0; i <= A1.actionsList.get(v2).size(); i++) {
					
					// We have a '+1' because once the pickup is inserted, the size is increased.
					for (int j = 0; j <= A1.actionsList.get(v2).size() + 1; j++) {
						
						Solution A_tmp = new Solution(A1);
						A_tmp.actionsList.get(v2).add(i, pickupAction);
						A_tmp.actionsList.get(v2).add(j, deliveryAction);
						A_tmp.computeCost();
						
						
						// We only keep the plan if it satisfies the constraints and is better than the current solution
						if (A_tmp.verifyConstraints() && A_tmp.cost < A.cost) {
							solutions.add(A_tmp);
						}
					
					}
					
				}
				
			}
			
		}
		
		return solutions;
		
	}
	
	
	/*
	 * We exchange the order of two given tasks
	 */
	public List<Solution> changingTaskOrder(Solution A, Vehicle vi) {
		
		List<Solution> solutions = new ArrayList<Solution>();
		
		for (Action a1 : A.actionsList.get(vi)) {
			for (Action a2 : A.actionsList.get(vi)) {
				if (!a1.equals(a2)) {
					
					Solution A_tmp = new Solution(A);
					int indexT1 = A_tmp.actionsList.get(vi).indexOf(a1);
					int indexT2 = A_tmp.actionsList.get(vi).indexOf(a2);
					
					A_tmp.actionsList.get(vi).remove(a1);
					A_tmp.actionsList.get(vi).remove(a2);
					
					// We have to insert the smallest index first, otherwise there are some out-of-bound issues.
					if (indexT1 < indexT2) {
						A_tmp.actionsList.get(vi).add(indexT1, a2);
						A_tmp.actionsList.get(vi).add(indexT2, a1);
					} else {
						A_tmp.actionsList.get(vi).add(indexT2, a1);
						A_tmp.actionsList.get(vi).add(indexT1, a2);
					}
					
					A_tmp.computeCost();
					
					// We only keep the plan if it satisfies the constraints and is better than the current solution
					if (A_tmp.verifyConstraints() && A_tmp.cost < A.cost) {
						solutions.add(A_tmp);
					}
			
				}
			}
		}
		
		return solutions;

	}

}

class Solution {

	protected HashMap<Vehicle, List<Action>> actionsList; // Used to store the actions of each vehicle
	protected Double cost;

	public static List<Vehicle> vehicles;
	public static List<Task> tasks;
	
	public Solution() {
		actionsList = new HashMap<Vehicle, List<Action>>();
		for (Vehicle vehicle : vehicles) {
			actionsList.put(vehicle, new ArrayList<Action>());
		}
	}

	public Solution(Solution parentSolution) {
		actionsList = new HashMap<Vehicle, List<Action>>();
		for (Vehicle vehicle : vehicles) {
			actionsList.put(vehicle, new ArrayList<Action>(parentSolution.actionsList.get(vehicle)));
		}
		computeCost();
	}

	/*
	 * Generate the plan for each vehicle for this solution
	 */
	public List<Plan> getPlan() {

		List<Plan> plans = new ArrayList<Plan>();
		
		for (Vehicle vehicle : vehicles) {
			
			List<Action> actions = actionsList.get(vehicle);
			City current = vehicle.homeCity();
			Plan plan = new Plan(current);
			
			for (Action action: actions) {
				
				for (City city : current.pathTo(action.city)) {
					plan.appendMove(city);
				}
				
				if (action.actionType.equals("pickup")) {
					plan.appendPickup(action.task);
				} else if (action.actionType.equals("delivery")) {
					plan.appendDelivery(action.task);
				} else {
					System.err.println("[Error] getPlan(): some action is neither a pickup nor a delivery action.");
				}
				
				current = action.city;
				
			}
			
			plans.add(plan);
			
			System.out.println("Vehicle " + (vehicle.id() + 1) + "'s cost is " + (plan.totalDistance() * vehicle.costPerKm())+" ("+actions.size()/2+" Tasks: "+ plan+")");
			
		}

		return plans;
	}

	
	void computeCost() {
		double newCost = 0.0;
		
		for (Vehicle vehicle : vehicles) {
			City currentCity = vehicle.homeCity();
			for (Action action : actionsList.get(vehicle)) {
				newCost += currentCity.distanceTo(action.city) * vehicle.costPerKm();
				currentCity = action.city;
			}
		}
		
		this.cost = newCost;
	}

	/**
	 * Verify the constraints.
	 * @return true if the constraints are fulfilled, false otherwise.
	 */
	 Boolean verifyConstraints() {

		/*
		 * Constraint 1
		 * We only accept if the vehicle can carry the tasks, at any moment
		 */		
		for (Vehicle vehicle : vehicles) {
			
			int carriedWeight = 0;
			
			for (Action action: actionsList.get(vehicle)) {
				
				if (action.actionType.equals("pickup")) {
					carriedWeight += action.task.weight;
				} else {
					carriedWeight -= action.task.weight;
				}
	
				if (carriedWeight > vehicle.capacity()) {
					return false;
				}
				
			}
			
		}
		
		
		/*
		 * Constraint 2
		 * Pickups actions of a task must be before corresponding deliveries, all picked up tasks must be delivered and all tasks available must be picked up.
		 */
		ArrayList<Task> availableTasks = new ArrayList<Task>(tasks);
		
		for (Vehicle vehicle : vehicles) {
			
			ArrayList<Task> stack = new ArrayList<Task>();
			
			for (Object obj : actionsList.get(vehicle)) {
				
				Action action = (Action) obj;
				
				if (action.actionType.equals("pickup")) {
					stack.add(action.task);
					availableTasks.remove(action.task);
				} else if (action.actionType.equals("delivery")) {
					if (!stack.remove(action.task)) return false;
				} else {
					System.err.println("[Error] verifyConstraints(): some action is neither a pickup nor a delivery action.");
				}
				
			}
			
			// All picked up tasks must be delivered
			if (!stack.isEmpty()) return false;
			
		}
		
		// Verify that there is no task left
		if (!availableTasks.isEmpty()) {
			return false;
		}

		return true;
	}
	 
	@Override
	public String toString() {
		String string = "";
		
		for (Vehicle vehicle : vehicles){
			string = string + "\n" + actionsList.get(vehicle);
		}
		
		return string;
	}
	 
}

class Action {
	
	protected Task task;
	protected String actionType;
	protected City city;
	
	public Action(Task task, String type) {
		this.task = task;
		actionType = type;
		if (actionType.equals("pickup")) {
			city = this.task.pickupCity;
		} else if (actionType.equals("delivery")) {
			city = this.task.deliveryCity;
		} else {
			System.err.println("[Error] Attempt to create an action that is not a pickup nor a delivery action.");
		}
	}
	
	@Override
	public String toString() {
		return actionType + " Task" + task.id + " in " + city;
	}
	
	public boolean equals(Object obj) {
		Action action = (Action) obj;
		return this.task.equals(action.task) && this.actionType.equals(action.actionType);
	}
}
