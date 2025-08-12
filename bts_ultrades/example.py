import clr
clr.AddReference("UltraDES")

from UltraDES import State, Event, Transition, DeterministicFiniteAutomaton, Controllability
from System.Collections.Generic import List
from bts_ultrades import BipartiteTransitionSystem

# --- Definição da planta ---
s0 = State("s0")
s1 = State("s1")
s2 = State("s2")
a = Event("a", Controllability.Controllable); setattr(a, "Observable", True)
b = Event("b", Controllability.Uncontrollable); setattr(b, "Observable", True)
c = Event("c", Controllability.Controllable); setattr(c, "Observable", True)

transitions = List[Transition]()
transitions.Add(Transition(s0, a, s1))
transitions.Add(Transition(s1, b, s2))
transitions.Add(Transition(s2, c, s0))

G = DeterministicFiniteAutomaton(transitions, s0, "PlantaG")

# --- Construção do BTS ---
bts = BipartiteTransitionSystem(G, name="BTS_Teste", forbidden_states=["s2"])
bts.report()
