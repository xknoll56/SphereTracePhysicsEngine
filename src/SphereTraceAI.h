#pragma once
#include "SphereTraceMath.h"

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


ST_StateMachine sphereTraceStateMachineConstruct(int numStates);

void sphereTraceStateMachineInsertState(ST_StateMachine* const pStateMachine, const ST_State* const pState);

void sphereTraceStateMachineStart(ST_StateMachine* const pStateMachine);

void sphereTraceStateMachineUpdate(ST_StateMachine* const pStateMachine);

ST_State sphereTraceStateMachineStateConstruct(int stateIndex, int numConnections);

ST_State sphereTraceStateMachineStateConstructWithStateFunctions(int stateIndex, int numConnections, void (*onUpdate)(), void (*onEnter)(), void (*onExit)(), ST_StateConnection* connections);


ST_StateConnection sphereTraceStateMachineStateConnectionConstruct(int stateIndex, int nextStateIndex, b32(*conditionalFunction)());

ST_StateConnection sphereTraceStateMachineStateConnectionConstructWithStates(const ST_State* pState, const ST_State* pNextState, b32(*conditionalFunction)());
