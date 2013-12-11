

//the list of imports
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import logist.agent.Agent;
import logist.behavior.AuctionBehavior;
import logist.plan.Plan;
import logist.simulation.Vehicle;
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
public class AuctionAgentDummy implements AuctionBehavior {

	private final static long TIMEOUT_BID = 5 * 1000;
	private final static long TIMEOUT_PLAN = 5 * 1000;
	
	private Topology topology;

	private Agent agent;
	private List<Vehicle> vehicles;
	private List<Task> tasks;
	private Solution2 currentPlans;
	private Solution2 futurePlans;
	
	private static String DUMB_TYPE = "Random";
	
	@Override
	public void setup(Topology topology, TaskDistribution distribution, Agent agent) {

		this.topology = topology;
		this.agent = agent;
		this.vehicles = agent.vehicles();
		this.tasks = new ArrayList<Task>();
		
		Solution2.vehicles = this.vehicles;
		
		currentPlans = new Solution2(new ArrayList<Task>());
		currentPlans.cost = 0.0;
		futurePlans = new Solution2(new ArrayList<Task>());
		futurePlans.cost = Double.POSITIVE_INFINITY;
	
	}

	@Override
	public void auctionResult(Task previous, int winner, Long[] bids) {
		
		if (winner == agent.id()) {
			this.tasks.add(previous);
			this.currentPlans = this.futurePlans;
		}
		
	}
	
	@Override
	public Long askPrice(Task task) {

		long timestart = System.currentTimeMillis();
		
		if (DUMB_TYPE.equals("Marginal")) {
			ArrayList<Task> futureTasks = new ArrayList<Task>(tasks);
			futureTasks.add(task);
			
			Double minCost = Double.POSITIVE_INFINITY;
			
			while (System.currentTimeMillis() < timestart + TIMEOUT_BID) {
				Solution2 plan = centralizedPlan(vehicles, futureTasks, 0.8);
				if (plan.cost < minCost) {
					minCost = plan.cost;
					this.futurePlans = plan;
				}
			}
			
			double marginalCost = futurePlans.cost - currentPlans.cost;
			
			return Math.max(0, Math.round(marginalCost));
			
		} else if (DUMB_TYPE.equals("Random")) {
			return Math.round(Math.random() * 5000);
			
		} else {
			return 0l;
		}
		
	}

	@Override
	public List<Plan> plan(List<Vehicle> vehicles, TaskSet tasks) {
		
		long timestart = System.currentTimeMillis();
		
		double income = 0;
		for (Task task : tasks) {
			income += task.reward;
		}
		
		Solution2 sol = centralizedPlan(vehicles, new ArrayList<Task>(tasks), 0.8);
		while (System.currentTimeMillis() < timestart + TIMEOUT_PLAN) {
			Solution2 plan = centralizedPlan(vehicles, new ArrayList<Task>(tasks), 0.8);
			if (plan.cost < sol.cost) {
				sol = plan;
			}
		}
		
		System.out.println("************** \tDummy: "+(income - sol.cost));
		
		return sol.getPlan();
	}

	private Solution2 centralizedPlan(List<Vehicle> vehicles, ArrayList<Task> tasks, double p) {

		Solution2.vehicles = vehicles;

		Solution2 Aold = new Solution2(tasks);
		Aold.cost = Double.POSITIVE_INFINITY;
		
		//The biggest vehicle handles tasks sequentially
		Solution2 A = selectInitialSolution2(vehicles, tasks);
		
		if (!A.verifyConstraints()) {
			System.err.println("At least one task is too big for the biggest capacity's vehicle!");
			System.exit(-1);
		}
		
		List<Solution2> N = null;

		int count = 0;
		
		//We continue while we improve
		while (count < 10000 && A.cost < Aold.cost) {
			Aold = new Solution2(A);
			
			N = chooseNeighbours(Aold, tasks, vehicles);
			
			//We also add the old state in order to prevent NullPointerExceptions if no neighbour is better
			N.add(Aold);
			
			//Select the best Solution2 among the neighbours (and the current Solution2)
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
	 * As an initial Solution2, we just take the vehicle with biggest capacity
	 * and assign all the tasks to it, sequentially.
	 * @param vehicles the list of vehicles
	 * @param tasks the liste of tasks
	 * @return an initial Solution2
	 */
	private Solution2 selectInitialSolution2(List<Vehicle> vehicles, List<Task> tasks) {

		Vehicle biggestVehicle = null;
		int maxCapacity = 0;
		for (Vehicle vehicle : vehicles) {
			if (vehicle.capacity() > maxCapacity) {
				maxCapacity = vehicle.capacity();
				biggestVehicle = vehicle;
			}
		}

		Solution2 initialSolution2 = new Solution2(tasks);

		for (Task task : tasks) {
			initialSolution2.actionsList.get(biggestVehicle).add(new Action(task, "pickup"));
			initialSolution2.actionsList.get(biggestVehicle).add(new Action(task, "delivery"));
		}

		initialSolution2.computeCost();

		return initialSolution2;
		
	}

	private List<Solution2> chooseNeighbours(Solution2 Aold, List<Task> tasks, List<Vehicle> vehicles) {

		List<Solution2> N = new ArrayList<Solution2>();
		
		for (Vehicle vi : vehicles) {
			
			if (!Aold.actionsList.get(vi).isEmpty()) {
				
				// Applying the changing vehicle operation:
				for (Vehicle vj : vehicles) {
					if (!vj.equals(vi)) {
						List<Solution2> As = changingVehicle(Aold, vi, vj);
						N.addAll(As);
					}
				}
				
				// Applying the changing task order operation:
				List<Solution2> As = changingTaskOrder(Aold, vi);
				N.addAll(As);
						
			}
			
		}

		return N;

	}

	/*
	 * We choose the best local Solution2. If multiple Solution2s are equally good, we choose one at random.
	 */
	private Solution2 localChoice(List<Solution2> N) {

		List<Solution2> bestSolution2s = new ArrayList<Solution2>();
		double leastCost = Double.POSITIVE_INFINITY;

		for (Solution2 Solution2 : N) {
			
			if (Solution2.cost < leastCost) {
				leastCost = Solution2.cost;
				bestSolution2s = new ArrayList<Solution2>();
				bestSolution2s.add(Solution2);
				
			} else if (Solution2.cost == leastCost) {
				bestSolution2s.add(Solution2);
			}
		}
	
		return bestSolution2s.get((int) (Math.random() * bestSolution2s.size()));
		
	}
	
	/* 
	 * We generate all the neighbours by giving one task handled by v1 and giving it to v2
	 */
	
	public List<Solution2> changingVehicle(Solution2 A, Vehicle v1, Vehicle v2) {
		
		List<Solution2> Solution2s = new ArrayList<Solution2>();
		
		//We can give any task of v1 to v2
		for(int actionIndex = 0; actionIndex < A.actionsList.get(v1).size(); actionIndex++) {
			
			Solution2 A1 = new Solution2(A);
			
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
						
						Solution2 A_tmp = new Solution2(A1);
						A_tmp.actionsList.get(v2).add(i, pickupAction);
						A_tmp.actionsList.get(v2).add(j, deliveryAction);
						A_tmp.computeCost();
						
						
						// We only keep the plan if it satisfies the constraints and is better than the current Solution2
						if (A_tmp.verifyConstraints() && A_tmp.cost < A.cost) {
							Solution2s.add(A_tmp);
						}
					
					}
					
				}
				
			}
			
		}
		
		return Solution2s;
		
	}
	
	
	/*
	 * We exchange the order of two given tasks
	 */
	public List<Solution2> changingTaskOrder(Solution2 A, Vehicle vi) {
		
		List<Solution2> Solution2s = new ArrayList<Solution2>();
		
		for (Action a1 : A.actionsList.get(vi)) {
			for (Action a2 : A.actionsList.get(vi)) {
				if (!a1.equals(a2)) {
					
					Solution2 A_tmp = new Solution2(A);
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
					
					// We only keep the plan if it satisfies the constraints and is better than the current Solution2
					if (A_tmp.verifyConstraints() && A_tmp.cost < A.cost) {
						Solution2s.add(A_tmp);
					}
			
				}
			}
		}
		
		return Solution2s;

	}

}

class Solution2 {

	protected HashMap<Vehicle, List<Action>> actionsList; // Used to store the actions of each vehicle
	protected Double cost;

	public static List<Vehicle> vehicles;
	public  List<Task> tasks;
	
	public Solution2(List<Task> tasks) {
		this.tasks = tasks;
		actionsList = new HashMap<Vehicle, List<Action>>();
		for (Vehicle vehicle : vehicles) {
			actionsList.put(vehicle, new ArrayList<Action>());
		}
	}

	public Solution2(Solution2 parentSolution2) {
		actionsList = new HashMap<Vehicle, List<Action>>();
		for (Vehicle vehicle : vehicles) {
			actionsList.put(vehicle, new ArrayList<Action>(parentSolution2.actionsList.get(vehicle)));
		}
		computeCost();
		tasks = new ArrayList<Task>(parentSolution2.tasks);
	}

	/*
	 * Generate the plan for each vehicle for this Solution2
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
		
		String string = "Tasks: "+tasks+"\n";
		string += "Cost: "+cost+"\n";
		string += "Vehicles' actions:";
		for (Vehicle vehicle : vehicles){
			string = string + "\n" + actionsList.get(vehicle);
		}
		
		return string;
	}
	 
}

//class Action {
//	
//	protected Task task;
//	protected String actionType;
//	protected City city;
//	
//	public Action(Task task, String type) {
//		this.task = task;
//		actionType = type;
//		if (actionType.equals("pickup")) {
//			city = this.task.pickupCity;
//		} else if (actionType.equals("delivery")) {
//			city = this.task.deliveryCity;
//		} else {
//			System.err.println("[Error] Attempt to create an action that is not a pickup nor a delivery action.");
//		}
//	}
//	
//	@Override
//	public String toString() {
//		return actionType + " Task" + task.id + " in " + city;
//	}
//	
//	public boolean equals(Object obj) {
//		Action action = (Action) obj;
//		return this.task.equals(action.task) && this.actionType.equals(action.actionType);
//	}
//}
