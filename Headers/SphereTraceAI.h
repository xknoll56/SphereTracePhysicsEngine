#pragma once

typedef struct ST_StateConnection
{
	int stateIndex;
	int nextStateIndex;
	b32 (*conditionalFunction)();
} ST_StateConnection;

typedef struct ST_State
{
	void (*onUpdate)();
	void (*onEnter)();
	void (*onExit)();
	int stateIndex;
	int numConnections;
	struct ST_StateConnection* connections;
} ST_State;


typedef struct ST_StateMachine
{
	ST_State* states;
	int numStates;
	int currentStateIndex;
	int startStateIndex;
} ST_StateMachine;


ST_StateMachine sphereTraceStateMachineConstruct(int numStates)
{
	ST_StateMachine stateMachine;
	stateMachine.numStates = numStates;
	stateMachine.states = malloc(sizeof(ST_State) * numStates);
	stateMachine.currentStateIndex = 0;
	stateMachine.startStateIndex = 0;
	return stateMachine;
}

void sphereTraceStateMachineInsertState(ST_StateMachine* const pStateMachine, const ST_State* const pState)
{
	pStateMachine->states[pState->stateIndex] = *pState;
}

void sphereTraceStateMachineStart(ST_StateMachine* const pStateMachine)
{
	pStateMachine->currentStateIndex = pStateMachine->startStateIndex;
	pStateMachine->states[pStateMachine->currentStateIndex].onEnter();
}
void sphereTraceStateMachineUpdate(ST_StateMachine* const pStateMachine)
{
	pStateMachine->states[pStateMachine->currentStateIndex].onUpdate();
	b32 nextStateTriggered = 0;
	int nextState;
	for (int i = 0; i < pStateMachine->states[pStateMachine->currentStateIndex].numConnections; i++)
	{
		if (pStateMachine->states[pStateMachine->currentStateIndex].connections[i].conditionalFunction())
		{
			nextStateTriggered = 1;
			nextState = pStateMachine->states[pStateMachine->currentStateIndex].connections[i].nextStateIndex;
			break;
		}
	}
	if (nextStateTriggered)
	{
		pStateMachine->states[pStateMachine->currentStateIndex].onExit();
		pStateMachine->currentStateIndex = nextState;
		pStateMachine->states[pStateMachine->currentStateIndex].onEnter();
	}
}

ST_State sphereTraceStateMachineStateConstruct(int stateIndex, int numConnections)
{
	ST_State state;
	state.numConnections = numConnections;
	state.connections = malloc(sizeof(ST_StateConnection) * numConnections);
	state.stateIndex = stateIndex;
	return state;
}

ST_State sphereTraceStateMachineStateConstructWithStateFunctions(int stateIndex, int numConnections, void (*onUpdate)(), void (*onEnter)(), void (*onExit)(), ST_StateConnection* connections)
{
	ST_State state;
	state.numConnections = numConnections;
	state.connections = (ST_StateConnection*)malloc(sizeof(ST_StateConnection) * numConnections);
	state.stateIndex = stateIndex;
	state.onUpdate = onUpdate;
	state.onEnter = onEnter;
	state.onExit = onExit;
	for (int i = 0; i < numConnections; i++)
	{
		state.connections[i] = connections[i];
	}
	return state;
}


ST_StateConnection sphereTraceStateMachineStateConnectionConstruct(int stateIndex, int nextStateIndex, b32 (*conditionalFunction)())
{
	ST_StateConnection stateConnection;
	stateConnection.stateIndex = stateIndex;
	stateConnection.nextStateIndex = nextStateIndex;
	stateConnection.conditionalFunction = conditionalFunction;
	return stateConnection;
}

ST_StateConnection sphereTraceStateMachineStateConnectionConstructWithStates(const ST_State* pState, const ST_State* pNextState, b32(*conditionalFunction)())
{
	ST_StateConnection stateConnection;
	stateConnection.stateIndex = pState->stateIndex;
	stateConnection.nextStateIndex = pNextState->stateIndex;
	stateConnection.conditionalFunction = conditionalFunction;
	return stateConnection;
}
