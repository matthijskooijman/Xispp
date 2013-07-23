#!/usr/bin/env python

# This script contains a state machine of the Xispp protocol. It can
# generate a dot graph containing the state machine for a single peer,
# or a dot graph with the combined state machine of two peers.
#
# To tweak the output generated, you can modify various parameter below.
#
# The dot output format was intended to be imported into the QFSM
# program. However, the extra simulation features of QFSM didn't really
# work with the type of state machine generated, turning QFSM into a
# over-sized dot viewer application. To view these graphs, you are
# better of using dot or xdot directly, since these also support
# newlines in labels (which QFSM doesn't).

######################################################
# Start of tweakable settings
######################################################

# Switch between showing a single peer FSM and the combined FSM of two
# peers
output_combined = True

# Everything below here only applies to the combined FSM

# Do any state merging at all?
merge_states = True
# Merge nodes if their state is the same but they have different
# outgoing transitions
very_aggressive_merge_samestate = True
# Merge nodes if their state is the same but they have different
# outgoing event transitions. Unused when
# very_aggressive_merge_samestate is set.
aggressive_merge_samestate = False
# Consider merging different states (provided they have the same
# outgoing transitions)
merge_different_state = False

# Remove boring states and merge all incoming and outgoing transitions
# of it.
# TODO: Fix this, breaks when there are loops within a single merged
# state
merge_transitions = False

# Model with one of the peers (initiall) set to disconnected.
check_disconnected = False
# Model connecting the channel at arbitrary moments. Nees
# check_disconnected = True to be meaningful
check_connect = False

# Set to False to remove event transitions from the output (they are
# still modeled)
output_event_transitions = True

######################################################
# End of tweakable settings
######################################################

import itertools
import collections
import operator

# Line readings
HIGH = 'High'
LOW = 'Low'
ANY = '*'

# Line drive states
LINE_IDLE = 'Idle (pull high)'
LINE_SIGNAL = 'Ack (drive high)'
LINE_REQ = 'Request (pull low)'

# Note: The poll delay now models the maximum number of time after its
# own state change that a peer might not be looking, while in reality
# this would indicate how long after a line state change the peer might
# not be looking. However, in practice this mechanism should work well
# enough to verify that it's ok to not pay attention at all in the idle
# and unavail states
class SingleState:
    def __init__(self, name, linestate, poll_time):
        self.name = name
        self.linestate = linestate
        self.poll_time = poll_time
        # All states are valid by default
        self.peer_states = None
        self.merge_with = [self]

    def __str__(self):
        return self.name


class Transition:
    def __init__(self, src, tgt, linestate, timer_set, timer_wait, is_event, comment):
        self.src = src
        self.tgt = tgt
        self.linestate = linestate
        self.timer_set = timer_set
        self.timer_wait = timer_wait
        self.is_event = is_event
        self.comment = comment

        self.output = output_event_transitions or not is_event

    def __str__(self):
        return self.comment

class CombinedTransition:
    def __init__(self, *ts):
        self.ts = ts

    def __str__(self):
        return ' / '.join(str(t) if t else ' - ' for t in self.ts)

    def any_event(self):
        return any(t is not None and t.is_event for t in self.ts)

    def needs_output(self):
        return all(t is None or t.output for t in self.ts)

    def allow_merge(self):
        # Don't merge transitions we'll not output to prevent confusing
        # output
        if not self.needs_output():
            return False
        return True

class WaitTransition:
    def __init__(self, time):
        self.time = time

    def __str__(self):
        return 'wait {}'.format(self.time)

    def any_event(self):
        return False

    def needs_output(self):
        return True

    def allow_merge(self):
        return True


def resolve_line(s1, s2):
    if LINE_SIGNAL in (s1.linestate, s2.linestate):
        return HIGH
    if LINE_REQ in (s1.linestate, s2.linestate):
        return LOW
    return HIGH

class CombinedState:
    def __init__(self, s1, t1, s2, t2, line_time):
        self.s1 = s1
        self.t1 = t1
        self.s2 = s2
        self.t2 = t2
        self.done = False
        self.line_time = line_time
        # Maps target state to the transition path (list of
        # CombinedTransition / WaitTransition objects)
        self.transitions = collections.defaultdict(set)
        self.linestate = resolve_line(s1, s2)
        self.preds = set()

    def is_valid(self):
        if self.s1.peer_states and self.s2 not in self.s1.peer_states:
            return False
        if self.s2.peer_states and self.s1 not in self.s2.peer_states:
            return False
        return True

    def add_combined_transition(self, tr1, tr2):
        new_s1 = tr1.tgt if tr1 else self.s1
        new_t1 = tr1.timer_set if tr1 and tr1.timer_set != UNCHANGED else self.t1
        new_s2 = tr2.tgt if tr2 else self.s2
        new_t2 = tr2.timer_set if tr2 and tr2.timer_set != UNCHANGED else self.t2
        new_linestate = resolve_line(new_s1, new_s2)
        new_line_time = 0 if new_linestate != self.linestate else self.line_time

        self.transitions[get_combined_state(new_s1, new_t1, new_s2, new_t2, new_line_time)].add((get_combined_trans(tr1, tr2),))

    def __str__(self):
        return self.name()

    def name(self):
        return '{}_{}_{}_{}'.format(self.s1, self.t1, self.s2, self.t2)

class MergedStates:
    def __init__(self, states):
        self.states = states
        self.linestate = self.states[0].linestate

    def label(self):
        timeless = set((s.s1, s.s2) for s in self.states)
        labels = []
        # When self.states contains a lot of duplicate times (but in
        # different combinations), it can help a lot to simplify them.
        # We repeatedly try to find the biggest "rectangle" over the matrix
        # that contains the valid times and then output each subsequent
        # rectangle as a pair of ranges
        for s1, s2 in timeless:
            # This is a set of coordinates that are present
            present = set((s.t1, s.t2, s.line_time) for s in self.states if s.s1 == s1 and s.s2 == s2)
            # Find out, for each axis separately, what all possible
            # values are (e.g., from 0 up to and including the max of
            # the present values)
            axises = [max(v) for v in zip(*present)]

            while present:
                best = [0, None, None]

                def try_axis_max(min_vals, max_vals, axises_left):
                    if not axises_left:
                        # Values were chosen for all axises,
                        # evaluate them
                        size_per_axis = [maxval - minval + 1 for minval, maxval in zip(min_vals, max_vals)]
                        size = reduce(operator.mul, size_per_axis)
                        if size > best[0]:
                            best[0] = size
                            best[1] = min_vals
                            best[2] = max_vals
                    else:
                        minval = min_vals[len(max_vals)]
                        axis_max = axises_left[0]
                        for maxval in range(minval, axis_max + 1):
                            # Check if expanding our range to include
                            # this new maximum doesn't include any
                            # positions that are not in present
                            # First find min,max pairs for all axises
                            # that we chose a value for already
                            pairs = zip(min_vals, max_vals)
                            for prefix in itertools.product(*(range(minval, maxval + 1) for minval, maxval in pairs)):
                                # Iterate all combinations for these
                                # axises, and add the value we just
                                # picked for the current axis, and just
                                # use the minval for all axises we still
                                # need to pick a maximum for.
                                pos = prefix + (maxval,) + min_vals[(len(max_vals) + 1):]
                                # Not present? Then there's no point in
                                # further increasing maxval
                                if not pos in present:
                                    maxval = -1
                                    break
                            if maxval == -1:
                                break
                            try_axis_max(min_vals, max_vals + (maxval,), axises_left[1:])

                for min_vals in itertools.product(*(range(maxval + 1) for maxval in axises)):
                    try_axis_max(min_vals, (), axises)

                # Remove the chosen option from the list of present
                # combinations
                (_, minvals, maxvals) = best
                for t1, t2, line_time in itertools.product(*(range(minval, maxval + 1) for minval, maxval in zip(minvals, maxvals))):
                    present.remove((t1, t2, line_time ))

                # Generate min-max labels
                label_ranges = []
                for minval, maxval in zip(minvals, maxvals):
                    if minval == maxval:
                        label_ranges.append(str(minval))
                    else:
                        label_ranges.append("{}-{}".format(minval, maxval))

                labels.append('{} {} / {} {} ({})'.format(s1, label_ranges[0],
                                                          s2, label_ranges[1],
                                                          label_ranges[2]))

        if any(not s.is_valid() for s in self.states):
            labels.append('#######################')
            labels.append('#       INVALID       #')
            labels.append('#######################')
        return '\\n'.join(labels)

    def name(self):
        return self.states[0].name()

    def __str__(self):
        return self.label()

#                              name             line state     poll time
IDLE =            SingleState('idle',              LINE_IDLE,     5)
INACTIVE =        SingleState('inactive',          LINE_IDLE,     5)
DATA_READY =      SingleState('data_ready',        LINE_REQ,      5)
SEE_ACK =         SingleState('see_ack',           LINE_REQ,      0)
SEND_ACK =        SingleState('send_ack',          LINE_SIGNAL,   0)
ACK_SENT =        SingleState('ack_sent',          LINE_IDLE,     0)
CHECK_COLLISION = SingleState('check_collision',   LINE_IDLE,     0)
DISCONNECTED =    SingleState('disconnected',      LINE_IDLE,     0)
SENDING_DELAY =   SingleState('sending_delay',     LINE_REQ,      0)
SENDING =         SingleState('sending',           LINE_IDLE,     0)
RECEIVING =       SingleState('receiving',         LINE_IDLE,     0)

# The polling time for idle/canceled/data ready is fairly arbitrary,
# it can be as big as needed (though it helps to reduce the number
# of states to have them all equal for some reason).

# Set the list of allowed peer states for these states. For all other
# states, any peer state is ok.
SENDING_DELAY.peer_states = [ACK_SENT, RECEIVING]
SENDING.peer_states = [RECEIVING, ACK_SENT]
RECEIVING.peer_states = [SENDING, SENDING_DELAY, SEE_ACK]

# Consider these states as "the same" for the purpose of merging
IDLE.merge_with.append(INACTIVE)
INACTIVE.merge_with.append(IDLE)

states = [IDLE, INACTIVE, DATA_READY, SEE_ACK,
          SEND_ACK, ACK_SENT, SENDING_DELAY, SENDING,
          RECEIVING, CHECK_COLLISION]

# The ack must be so long that a DATA_READY peer will always see it
SEND_ACK_TIME = DATA_READY.poll_time + 1
# After seeing the end of the ack, wait a bit before letting the line go
# idle again, so we are sure the peer knows we're still in the request
# state.
SENDING_DELAY_TIME = 1

# When we cancel a request because we will be busy for a while
# (when receiving data through another channel, for example), we
# can't start a new request too soon, since then there will be
# a chance that our idle time coincides exactly with our peer's ack,
# meaning they'll think we saw their ack while we really weren't
# listening.
CANCEL_TIME = SEND_ACK_TIME + 1

# Check for collision when we're sure that our peer would have sent
# an ack if it wasn't busy. This time can be smaller as well, but
# that increases the number of states
# This means we can make the idle poll time arbitrarily large,
# without really having to increase this time as well.
COLLISION_TIME = IDLE.poll_time + 1

UNCHANGED = -1

transitions = [
        #          from_state,        to_state,           line,    set timer            timer wait    event        comment
        Transition(IDLE,              DATA_READY,         HIGH,    COLLISION_TIME,      True,         True,   'data_ready'),
        Transition(IDLE,              SEND_ACK,           LOW,     SEND_ACK_TIME,       False,        False,  'see_request'),
        Transition(IDLE,              INACTIVE,           ANY,     UNCHANGED,           False,        True,   'inactive'),
        Transition(INACTIVE,          IDLE,               ANY,     UNCHANGED,           False,        True,   'active'),
        # Receiver states
        Transition(SEND_ACK,          ACK_SENT,           ANY,     UNCHANGED,           True,         False,  'ack_done'),
        Transition(ACK_SENT,          IDLE,               HIGH,    0,                   False,        False,  'peer_canceled'),
        Transition(ACK_SENT,          RECEIVING,          LOW,     UNCHANGED,           False,        False,  'comm_start'),
        # Sender states
        Transition(DATA_READY,        CHECK_COLLISION,    LOW,     UNCHANGED,           True,         False,  'check_collision'),
        Transition(DATA_READY,        INACTIVE,           ANY,     CANCEL_TIME,         False,        True,   'cancel'),
        Transition(DATA_READY,        SEE_ACK,            HIGH,    UNCHANGED,           False,        False,  'incoming_ack'),
        Transition(CHECK_COLLISION,   SEND_ACK,           LOW,     SEND_ACK_TIME,       False,        False,  'collision'),
        Transition(CHECK_COLLISION,   DATA_READY,         HIGH,    COLLISION_TIME,      False,        False,  'no_collision'),
        Transition(SEE_ACK,           SENDING_DELAY,      LOW,     SENDING_DELAY_TIME,  False,        False,  'ack_ok'),
        Transition(SENDING_DELAY,     SENDING,            ANY,     UNCHANGED,           True,         False,  'comm_start'),
]

# This tries to model the effect of connecting two peers when either or
# both of them are not in the idle state.  These are all the states a
# peer could get into while the other is disconnected. This is not a
# perfect modeling, since the timer is always set to 0 upon connection,
# which isn't realistic.
if output_combined and check_connect:
    transitions.extend([
        Transition(DISCONNECTED,      IDLE,               ANY,     0,                   False,        True,   'connect'),
        Transition(DISCONNECTED,      DATA_READY,         ANY,     0,                   False,        True,   'connect'),
        Transition(DISCONNECTED,      CHECK_COLLISION,    ANY,     0,                   False,        True,   'connect'),
    ])


def print_fsm():
    """
    Prints the FSM of a single peer.
    """
    non_terminals = set(t.src for t in transitions)
    for s in states:
        shape = 'circle' if s in non_terminals else 'doublecircle'
        print("\t{} [ label = \"State: {}\\nLine: {}\\nPoll: {}\" shape = {} ];".format(s, s, s.linestate, s.poll_time, shape))

    for t in transitions:
        label = ""
        if t.is_event:
            label += "!"
        label += t.comment
        label += "\\n"
        label += "Line == {}".format(t.linestate)
        if t.timer_wait:
            label += " && timer == 0"
        if t.timer_set != UNCHANGED:
            label += " | set timer = {}".format(t.timer_set)

        print("\t{} -> {} [label = \"{}\"];".format(t.src, t.tgt, label))

def transitions_from_state(state, timer, line_time, linestate):
    options = []
    for t in transitions:
        if state == t.src and (linestate == t.linestate or t.linestate == ANY):
            if timer != 0 and t.timer_wait:
                # Not available yet
                continue

            options.append(t)

    return options

def do_merge(merge, merge_into, cs_to_ms):
    # Merge states
    merge_into.states.extend(merge.states)

    # Update mapping
    for s in merge.states:
        cs_to_ms[s] = merge_into

def try_merge(merge, merge_into, same_state, cs_to_ms):
    if merge == merge_into:
        return False;

    if merge_into.linestate != merge.linestate:
        return False

    if same_state and very_aggressive_merge_samestate:
        # Don't check successors, just merge all combined states with
        # the same substates but different timing info
        do_merge(merge, merge_into, cs_to_ms)
        return True
    if same_state and aggressive_merge_samestate:
        # Merge nodes if they have identical non-event successors
        def non_event(path):
            if isinstance(path[0], WaitTransition):
                return True

            # If either transition is actually happening
            # (i.e. has a comment) and does not have an
            # associated event, then this is a non-event
            # transition
            #return any(single_t is not None and not single_t.is_event
            #           for single_t in path[0].ts)
            return all(single_t is None or not single_t.is_event
                       for single_t in path[0].ts)
        succ_into = set(cs_to_ms[tgt] for s in merge_into.states for (tgt, paths) in s.transitions.items() for path in paths if not path[0].any_event())
        succ = set(cs_to_ms[tgt] for s in merge.states for (tgt, paths) in s.transitions.items() for path in paths if not path[0].any_event())
    else:
        # Merge nodes if they have identical successors
        succ_into = set(cs_to_ms[tgt] for s in merge_into.states for tgt in s.transitions.keys())
        succ = set(cs_to_ms[tgt] for s in merge.states for tgt in s.transitions.keys())

    # If the nodes have the same (non-event) successors, ignoring any
    # transitions between the two nodes and self-transitions, merge them
    if (succ_into - set([merge, merge_into])) == (succ - set([merge, merge_into])):
        # Do the merge and let our caller remove the old merged state
        do_merge(merge, merge_into, cs_to_ms)
        return True

    return False

def try_merge_into_all(merge, merged_states, cs_to_ms):
    merge_single_states = set()
    for s in merge.states:
        merge_single_states |= set(itertools.product(s.s1.merge_with, s.s2.merge_with))
        merge_single_states |= set(itertools.product(s.s2.merge_with, s.s1.merge_with))

    for merge_into in merged_states:
        merge_into_single_states = set((s.s1, s.s2) for s in merge_into.states)
        same_state = bool(merge_single_states & merge_into_single_states)

        if not merge_different_state and not same_state:
            continue

        if try_merge(merge, merge_into, same_state, cs_to_ms):
            return True

    return False

wait_transitions = {}
def get_wait(time):
    if not time in wait_transitions:
        wait_transitions[time] = WaitTransition(time)
    return wait_transitions[time]

def merge_waits(first, second):
    if isinstance(first[-1], WaitTransition) and isinstance(second[0], WaitTransition):
        wait = get_wait(first[-1].time + second[0].time)
        return first[:-1] + (wait,) + second[1:]
    return first + second

combined_states = {}
def get_combined_state(s1, t1, s2, t2, line_time, create = True):
    # Cap the line time on the biggest poll time
    line_time = min(line_time, max(s1.poll_time, s2.poll_time))

    if (s1, t1, s2, t2, line_time) not in combined_states:
        if create:
            # No such state yet, create a new one
            s = CombinedState(s1, t1, s2, t2, line_time)
            combined_states[(s1, t1, s2, t2, line_time)] = s
        else:
            return None
    return combined_states[(s1, t1, s2, t2, line_time)]

combined_transitions = {}
def get_combined_trans(*ts):
    if not ts in combined_transitions:
        combined_transitions[ts] = CombinedTransition(*ts)
    return combined_transitions[ts]

def print_combined_fsm():
    done = set()
    queue = set()

    if check_disconnected:
        queue.add(get_combined_state(IDLE, 0, DISCONNECTED, 0, 0))
    else:
        queue.add(get_combined_state(IDLE, 0, IDLE, 0, 0))

    while queue:
        state = queue.pop()
        if state.done:
            continue

        s1 = state.s1
        t1 = state.t1
        s2 = state.s2
        t2 = state.t2
        line_time = state.line_time

        trans1 = transitions_from_state(s1, t1, line_time, state.linestate)
        trans2 = transitions_from_state(s2, t2, line_time, state.linestate)

        poll_time_left1 = s1.poll_time - line_time
        poll_time_left2 = s2.poll_time - line_time

        may_wait = True

        for tr1 in trans1:
            state.add_combined_transition(tr1, None)

            # If there is a non-event triggered transition available and the
            # poll time for the current state has been reached, we must
            # this transition must be taken and no waiting is allowed
            if not tr1.is_event and poll_time_left1 <= 0:
                may_wait = False

        for tr2 in trans2:
            state.add_combined_transition(None, tr2)
            # If there is a non-event triggered transition available and the
            # poll time for the current state has been reached, we must
            # this transition must be taken and no waiting is allowed
            if not tr2.is_event and poll_time_left2 <= 0:
                may_wait = False


        for tr1, tr2 in itertools.product(trans1, trans2):
            state.add_combined_transition(tr1, tr2)

        # Neither of the peers have any unconditional (without event)
        # transitions right now, so we might need to insert a wait
        # transition.
        if may_wait:
            non_zero = filter(lambda d: d > 0, [t1, t2, poll_time_left1, poll_time_left2])
            if non_zero:
                delay = min(non_zero)

                # If only one peer gets an new transition after waiting,
                # we only increment the time for that peer, to prevent
                # indefinitely incrementing the time for the other peer.

                state.transitions[get_combined_state(s1, max(0, t1 - delay), s2, max(0, t2 - delay), line_time + delay)].add((get_wait(delay),))

        state.done = True

        for succ in state.transitions:
            succ.preds.add(state)
            if not succ.done:
                queue.add(succ)

    merged_states = []
    # a map from each CombinedState to its MergedState
    cs_to_ms = {}

    for ms in merged_states:
        for cs in ms.states:
            cs_to_ms[cs] = ms

    if not merge_states:
        # We're not merging states, so create dummy MergedStates
        # containing just one state each
        for cs in combined_states.values():
            ms = MergedStates([cs])
            merged_states.append(ms)
            cs_to_ms[cs] = ms
    else:
        while combined_states:
            _, cs = combined_states.popitem()
            if cs.s1 == cs.s2 or not (cs.s2, cs.t2, cs.s1, cs.t1, cs.line_time) in combined_states:
                # Both peer states are the same, just add this combined
                # state on its own
                ms = MergedStates([cs])
            else:
                # Peer states are different, merge this combinedstate with
                # its mirror state right away
                other = combined_states.pop((cs.s2, cs.t2, cs.s1, cs.t1, cs.line_time))
                ms = MergedStates([cs, other])
                cs_to_ms[other] = ms
            merged_states.append(ms)
            cs_to_ms[cs] = ms

    def try_remove(remove):
        # The result is better if we don't remove this state
        for s in remove.states:
            if (s.s1, s.t1, s.s2, s.t2) == (IDLE, 0, IDLE, 0):
                return False

        for s in remove.states:
            if any(not t.allow_merge() for paths in s.transitions.values() for path in paths for t in path):
                return False
            for pred in s.preds:
                if any(not t.needs_output() for path in pred.transitions[s] for t in path):
                    return False

            succs = set(cs_to_ms[tgt] for tgt in s.transitions)
            preds = set(cs_to_ms[pred] for pred in s.preds)

            # Don't remove begin and end states
            if not preds or not succs:
                return False

            # Don't remove a state if it would result in more or just as
            # much transitions than we have now. i.e, only remove if
            # either the number of predecessors or successors is 1
            if len(preds) * len(succs) >= len(preds) + len(succs):
                return False

            # But don't remove this MergedState if any of its
            # CombinedStates has a self-transition (which could occur
            # when a previous removal happened from a cycle). This
            # self-transition would be lost in the below removal.
            if any(tgt == s for tgt in s.transitions):
                return False

        # Cut away each of the merged CombinedStates separately
        for s in remove.states:
            for pred in s.preds:
                for target, paths in pred.transitions.items():
                    if target == s:
                        del pred.transitions[target]
                        for remove_target, remove_paths in s.transitions.items():
                            for remove_path in remove_paths:
                                for path in paths:
                                    remove_target.preds.add(pred)
                                    pred.transitions[remove_target].add(merge_waits(path, remove_path))

            for remove_target, remove_paths in s.transitions.items():
                remove_target.preds.remove(s)

        return True

    if merge_states:
        merged = True
        while merged:
            merged = False
            for ms in reversed(merged_states):
                if try_merge_into_all(ms, merged_states, cs_to_ms):
                    merged = True
                    merged_states.remove(ms)

    if merge_transitions:
        changed = True
        while changed:
            changed = False
            for remove in reversed(merged_states):
                if try_remove(remove):
                    merged_states.remove(remove)
                    changed = True


    #for s in combined_states.get((s1, s2), []):
    #    if s.transitions == new_trans or not s.done:
    #        s.times.append((t1, t2))
    #        merged = True

    num_transitions = 0
    num_states = 0
    #for ss in combined_states.values():
    for ms in merged_states:
        num_states += 1

        # Build a list of paths per target
        paths_per_target = collections.defaultdict(set)
        for cs in ms.states:
            for (target, paths) in cs.transitions.items():
                for path in paths:
                    if all(t.needs_output() for t in path):
                        paths_per_target[cs_to_ms[target]].add(path)

        # And output one edge for each target, joining the multiple
        # possible paths in the label
        for target_ms, paths in paths_per_target.items():
            label = '\\n'.join(' -> '.join(map(str, path)) for path in paths)
            print("\t{} -> {} [label = \"{}\"];".format(ms.name(), target_ms.name(), label))
            num_transitions += 1

        shape = 'circle' if paths_per_target else 'doublecircle'
        print("\t{} [ shape = {} label =\"{}\\n{}\"  ];".format(ms.name(), shape, ms.label(), ms.linestate))

    print("\t// States: {} Transitions: {}".format(num_states, num_transitions))

if output_combined:
    print("digraph two_peers {")
else:
    print("digraph single_peer {")

print("\trankdir=LR;")
print("\tsize=\"8,5\"")
print("\tnode [shape = circle];");

if output_combined:
    print_combined_fsm()
else:
    print_fsm()

print("}")
