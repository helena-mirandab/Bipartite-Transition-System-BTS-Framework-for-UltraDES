import clr
clr.AddReference("UltraDES")

from UltraDES import State, Event, Transition, DeterministicFiniteAutomaton, Controllability, Marking
from System.Collections.Generic import List
from itertools import combinations
from collections import deque

class BipartiteTransitionSystem:
    def __init__(self, G, name="BTS", forbidden_states=None):
        self.plant = G
        self.name = name

        # --- Particiona eventos ---
        sigma_events = list(G.Events)
        self.Sigma = set(sigma_events)
        self.controllable_events = {e for e in sigma_events if e.Controllability == Controllability.Controllable}
        self.uncontrollable_events = self.Sigma - self.controllable_events
        self.observable_events = {e for e in sigma_events if getattr(e, "Observable", True)}
        self.unobservable_events = self.Sigma - self.observable_events

        # --- Índice de transições da planta: origem -> { evento -> destino } ---
        trans_dict = {}
        for t in G.Transitions:
            o, ev, d = t.Origin, t.Trigger, t.Destination
            if o not in trans_dict:
                trans_dict[o] = {}
            trans_dict[o][ev] = d

        # --- Mapas de estados do BTS ---
        self._y_state_map = {}
        self._z_state_map = {}
        self._state_info_map = {}

        # --- Estado inicial y0 = {x0} ---
        x0 = G.InitialState
        y0_set = frozenset([x0])
        y0 = State(f"Y:{{{x0.Alias}}}", Marking.Unmarked)
        self._y_state_map[y0_set] = y0
        self._state_info_map[y0] = ("Y", y0_set, None)
        self.y0 = y0

        # --- Conjunto Γ: todas as decisões sobre Σ_c ---
        gamma_set_list = []
        controllable_list = list(self.controllable_events)
        for r in range(len(controllable_list) + 1):
            for combo in combinations(controllable_list, r):
                gamma_set_list.append(frozenset(combo))

        self.Gamma = set()
        self._gamma_event_map = {}
        for gamma in gamma_set_list:
            gname = f"Gamma{{{','.join(sorted(e.Alias for e in gamma))}}}" if gamma else "Gamma{}"
            gev = Event(gname, Controllability.Controllable)
            self._gamma_event_map[gamma] = gev
            self.Gamma.add(gev)

        # --- Construção BFS do BTS ---
        queue = deque()
        queue.append(("Y", y0_set))
        visited_y = {y0_set}
        visited_z = set()
        transitions_bts = List[Transition]()

        while queue:
            kind, key = queue.popleft()

            # ===== Y -> Z : aplica γ e calcula fecho não-observável =====
            if kind == "Y":
                Y_set = key
                Y_state = self._y_state_map[Y_set]

                for gamma, gamma_event in self._gamma_event_map.items():
                    if not all(
                        any((s in trans_dict) and (e in trans_dict[s]) for s in Y_set)
                        for e in gamma
                    ):
                        continue

                    # (2) Fechamento por eventos não-observáveis sob γ
                    closure = set(Y_set)
                    changed = True
                    while changed:
                        changed = False
                        for s in list(closure):
                            for e in self.unobservable_events:
                                # controlável fora de γ => desabilitado
                                if (e in self.controllable_events) and (e not in gamma):
                                    continue
                                if (s in trans_dict) and (e in trans_dict[s]):
                                    ns = trans_dict[s][e]
                                    if ns not in closure:
                                        closure.add(ns)
                                        changed = True

                    I_z = frozenset(closure)

                    # (3) Checagem de VIABILIDADE do Z:
                    # existe algum evento observável permitido por γ habilitado em algum s ∈ I_z?
                    viable = False
                    for e in self.observable_events:
                        # se observável é controlável, precisa estar em γ
                        if (e in self.controllable_events) and (e not in gamma):
                            continue
                        if any((s in trans_dict) and (e in trans_dict[s]) for s in I_z):
                            viable = True
                            break
                    if not viable:
                        continue  # não cria Z nem transição Y --γ--> Z

                    # (4) Cria/recupera Z e adiciona transição Y --γ--> Z
                    z_key = (I_z, gamma)
                    if z_key not in self._z_state_map:
                        z_alias = f"Z:{{{','.join(sorted(s.Alias for s in I_z))}}}|{{{','.join(sorted(e.Alias for e in gamma))}}}"
                        Z_state = State(z_alias, Marking.Unmarked)
                        self._z_state_map[z_key] = Z_state
                        self._state_info_map[Z_state] = ("Z", I_z, gamma)
                        visited_z.add(z_key)
                        queue.append(("Z", z_key))
                    else:
                        Z_state = self._z_state_map[z_key]

                    transitions_bts.Add(Transition(Y_state, gamma_event, Z_state))

            # ===== Z -> Y : ocorre um evento observável =====
            elif kind == "Z":
                I_z, gamma = key
                Z_state = self._z_state_map[key]

                for e in self.observable_events:
                    # Evento observável controlável só pode ocorrer se e ∈ γ
                    if (e in self.controllable_events) and (e not in gamma):
                        continue

                    next_set = set()
                    for s in I_z:
                        if (s in trans_dict) and (e in trans_dict[s]):
                            next_set.add(trans_dict[s][e])

                    if not next_set:
                        continue

                    Y_next = frozenset(next_set)
                    if Y_next not in self._y_state_map:
                        y_alias = f"Y:{{{','.join(sorted(s.Alias for s in Y_next))}}}"
                        Y_state_new = State(y_alias, Marking.Unmarked)
                        self._y_state_map[Y_next] = Y_state_new
                        self._state_info_map[Y_state_new] = ("Y", Y_next, None)
                        visited_y.add(Y_next)
                        queue.append(("Y", Y_next))
                    else:
                        Y_state_new = self._y_state_map[Y_next]

                    transitions_bts.Add(Transition(Z_state, e, Y_state_new))

        # --- Armazena e compõe o DFA do BTS no UltraDES ---
        self.Q_Y = set(self._y_state_map.values())
        self.Q_Z = set(self._z_state_map.values())
        self.automaton = DeterministicFiniteAutomaton(transitions_bts, self.y0, name)

        # --- Estados proibidos (opcional) ---
        self.forbidden_set = set()
        if forbidden_states:
            for f in forbidden_states:
                if isinstance(f, str):
                    for s in G.States:
                        if s.Alias == f:
                            self.forbidden_set.add(s)
                elif hasattr(f, "Alias"):
                    self.forbidden_set.add(f)

    # ===== Utilitários =====
    def check_deadlock_free(self):
        origin_states = {t.Origin for t in self.automaton.Transitions}
        return all(st in origin_states for st in self.automaton.States)

    def check_property_satisfaction(self):
        if not self.forbidden_set:
            return True
        for y_set in self._y_state_map:
            if any(s in self.forbidden_set for s in y_set):
                return False
        for (z_set, _) in self._z_state_map:
            if any(s in self.forbidden_set for s in z_set):
                return False
        return True

    def print_transitions(self):
        print("Transições do BTS:")
        for t in self.automaton.Transitions:
            print(f"{t.Origin.Alias}  --{t.Trigger.Alias}--> {t.Destination.Alias}")

    def to_dot(self):
        lines = [f'digraph {self.name} {{']
        lines.append('  node [shape=rectangle, style=filled, fillcolor=lightblue];')
        lines += [f'  "{y.Alias}" [shape=rectangle];' for y in self.Q_Y]
        lines += [f'  "{z.Alias}" [shape=ellipse];' for z in self.Q_Z]
        lines.append(f'  "{self.y0.Alias}" [peripheries=2];')
        for t in self.automaton.Transitions:
            lines.append(f'  "{t.Origin.Alias}" -> "{t.Destination.Alias}" [label="{t.Trigger.Alias}"];')
        lines.append("}")
        return "\n".join(lines)

# ---------- Helpers de inspeção ----------
    def y_aliases(self):
        """Lista dos rótulos (Alias) dos estados Y."""
        return [s.Alias for s in self.Q_Y]

    def z_aliases(self):
        """Lista dos rótulos (Alias) dos estados Z."""
        return [s.Alias for s in self.Q_Z]

    def transitions_list(self):
        """Lista [(orig, evento, dest)] com aliases, útil para depuração/prints."""
        return [(t.Origin.Alias, t.Trigger.Alias, t.Destination.Alias)
                for t in self.automaton.Transitions]

    def num_transitions(self):
        """Quantidade de transições no BTS (converte o IEnumerable para lista)."""
        return len(list(self.automaton.Transitions))

    def report(self, show_transitions=True, limit=None, title="Resumo do BTS"):
        """
        Imprime um resumo padronizado do BTS.
        - show_transitions: imprime as transições
        - limit: limita quantas transições mostrar (None = todas)
        """
        print(f"{title}")
        print("Estados Y:", self.y_aliases())
        print("Estados Z:", self.z_aliases())
        print("Deadlock-free?", self.check_deadlock_free())
        print("Segurança satisfeita?", self.check_property_satisfaction())
        print("Total de transições:", self.num_transitions())

        if show_transitions:
            print("Transições do BTS:")
            count = 0
            for (o, ev, d) in self.transitions_list():
                print(f"{o}  --{ev}--> {d}")
                count += 1
                if limit is not None and count >= limit:
                    print(f"... (+{self.num_transitions() - count} outras)")
                    break

__all__ = ["BipartiteTransitionSystem"]
                    