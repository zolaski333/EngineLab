#!/bin/sh
# Runs a GUI test on Linux, headless or not.
#
# EngineLab.AudioWorkshop constructs real JUCE components, so it needs an X
# display. A bare Xvfb is not enough: JUCE 8 looks up the `_NET_WM_*` atoms
# with `XInternAtom(..., only_if_exists = True)` and, when no window manager
# has ever interned them, issues an `XChangeProperty` with atom 0 and dies
# with `BadAtom` before the first assertion runs. A window manager interns
# them, which is also closer to the desktop the test's assertions describe.
#
# Usage: run-gui-test.sh <command> [args...]

if [ "$#" -eq 0 ]; then
    echo "run-gui-test.sh: no command given" >&2
    exit 2
fi

# Does $DISPLAY point at a screen some window manager has already claimed?
# `_NET_SUPPORTED` on the root window is what answers that: a bare X server
# does not carry it, and interning it is the whole reason this wrapper exists.
# Without xprop the question cannot be asked, so answer no and bring our own.
display_has_window_manager() {
    [ -n "$DISPLAY" ] || return 1
    command -v xprop > /dev/null 2>&1 || return 1
    # The output is what must be tested, not the exit status: xprop returns 0
    # for a property that does not exist, printing "no such atom on any
    # window". Reading the status instead is why the wait loop below never
    # actually waited for the window manager, and it would have made this
    # whole check answer yes on a bare server.
    xprop -root _NET_SUPPORTED 2>/dev/null | grep -q '_NET_SUPPORTED(ATOM)'
}

# Second stage: already inside the virtual server, so DISPLAY points at it and
# the window manager can be started on it.
if [ "$1" = "--inside-virtual-server" ]; then
    shift
    matchbox-window-manager -use_titlebar no > /dev/null 2>&1 &
    manager=$!
    # Wait for it to claim the screen; `_NET_SUPPORTED` appears once it has.
    if command -v xprop > /dev/null 2>&1; then
        attempt=0
        while [ "$attempt" -lt 50 ]; do
            display_has_window_manager && break
            attempt=$((attempt + 1))
            sleep 0.1
        done
    else
        sleep 2
    fi
    "$@"
    status=$?
    kill "$manager" 2> /dev/null
    wait "$manager" 2> /dev/null
    exit "$status"
fi

# A display the developer is already sitting in front of has a window manager
# of its own: use it rather than hiding the window in a virtual server.
#
# Being *set* is not enough, and assuming it was is what made CI fail while
# every local run passed: the hosted Linux runner exports a DISPLAY with no
# window manager behind it, so the test went straight to the BadAtom this
# wrapper was written to avoid, in 0.49 s. WSLg, which does run one, hid the
# bug locally. Ask the display, do not trust the variable.
if display_has_window_manager; then
    exec "$@"
fi

if ! command -v xvfb-run > /dev/null 2>&1; then
    echo "run-gui-test.sh: no window-managed display and no xvfb-run, so this" \
         "test will probably fail with BadAtom; see CONTRIBUTING.md" >&2
    exec "$@"
fi

if ! command -v matchbox-window-manager > /dev/null 2>&1; then
    echo "run-gui-test.sh: no window manager found, so this test will probably" \
         "fail with BadAtom; install matchbox-window-manager (CONTRIBUTING.md)" >&2
    # -a picks a free server number, so parallel ctest jobs cannot collide.
    exec xvfb-run -a "$@"
fi

exec xvfb-run -a "$0" --inside-virtual-server "$@"
