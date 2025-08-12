import clr
# Se o UltraDES.dll estiver no PATH/.NET, basta:
clr.AddReference("UltraDES")
# Ou informe caminho absoluto: clr.AddReference("/caminho/para/UltraDES.dll")

from UltraDES import State, Event, Transition, DeterministicFiniteAutomaton, Controllability, Marking
from System.Collections.Generic import List


# -------- Planta G de exemplo --------
s0 = State("s0")
s1 = State("s1")
s2 = State("s2")

a = Event("a", Controllability.Controllable); setattr(a, "Observable", True)
b = Event("b", Controllability.Uncontrollable); setattr(b, "Observable", True)
c = Event("c", Controllability.Controllable); setattr(c, "Observable", True)

T = List[Transition]()
T.Add(Transition(s0, a, s1))
T.Add(Transition(s1, b, s2))
T.Add(Transition(s2, c, s0))

G = DeterministicFiniteAutomaton(T, s0, "PlantaG")

# -------- Constrói BTS --------
bts = BipartiteTransitionSystem(G, name="BTS_Teste", forbidden_states=["s2"])

# -------- Relatório --------
bts.report(show_transitions=True)
